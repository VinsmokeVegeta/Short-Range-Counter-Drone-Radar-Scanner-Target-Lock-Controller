#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>

/* ---------------- Pin Definitions ---------------- */
#define PIN_BUZZER        3U    /* Timer2 PWM tone output              */
#define PIN_LED_RED       4U    /* Threat Alert Indicator              */
#define PIN_LED_GREEN     5U    /* Normal Perimeter Scan Indicator     */
#define PIN_ECHO          6U    /* HC-SR04 Echo Pulse Input            */
#define PIN_TRIG          7U    /* HC-SR04 10 us Trigger Pulse Output  */
#define PIN_SERVO         9U    /* Timer1 Hardware PWM Servo Control   */

/* ---------------- Radar & Control Parameters ---------------- */
#define ANGLE_MIN_DEG     15
#define ANGLE_MAX_DEG     165
#define ANGLE_STEP_DEG    5

#define LOCK_RANGE_CM     120U  /* Perimeter acquisition threshold     */
#define ALERT_ENTER_CM    50U   /* High-threat entry threshold         */
#define ALERT_EXIT_CM     55U   /* 5 cm hysteresis prevents chattering */
#define MISS_LIMIT        3U    /* Consecutive misses before unlock    */

#define SCAN_PERIOD_MS    120UL /* Non-blocking servo step interval    */
#define TRACK_PERIOD_MS   150UL /* Closed-loop lock sampling interval  */
#define ALARM_BLINK_MS    200UL /* Threat buzzer/LED pulse interval    */

/* ---------------- 3-State Tracking FSM ---------------- */
typedef enum {
  STATE_SCAN = 0,
  STATE_TARGET_LOCK,
  STATE_THREAT_ALERT
} radar_state_t;

/* ---------------- Hardware Drivers ---------------- */
static Servo             g_turret;
static LiquidCrystal_I2C g_lcd(0x27, 16, 2);

/* ---------------- Static Runtime State ---------------- */
static radar_state_t g_state        = STATE_SCAN;
static int16_t       g_azimuth      = ANGLE_MIN_DEG;
static int8_t        g_sweep_dir    = 1;
static uint16_t      g_curr_dist_cm = 0;
static uint16_t      g_prev_dist_cm = 0;
static uint8_t       g_miss_count   = 0;

static uint32_t      g_last_step_ms  = 0;
static uint32_t      g_last_alarm_ms = 0;
static bool          g_alarm_phase   = false;

/* ---------------- Ultrasonic ToF Measurement ---------------- */
static uint16_t read_raw_distance_cm(void) {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  /* 20 ms timeout (~340 cm max) prevents long CPU blocking */
  unsigned long duration_us = pulseIn(PIN_ECHO, HIGH, 20000UL);
  if (duration_us == 0UL) {
    return 999U;  /* Out of range / no echo */
  }

  /* Speed of sound = 0.0343 cm/us; divide by 2 for round-trip ToF */
  return (uint16_t)((duration_us * 343UL) / 20000UL);
}

/* 3-Sample Median Filter rejects single-sample acoustic spikes */
static uint16_t read_filtered_distance_cm(void) {
  uint16_t s[3];
  for (uint8_t i = 0; i < 3U; i++) {
    s[i] = read_raw_distance_cm();
    delayMicroseconds(200);
  }

  /* Sort 3 values in place and return middle element */
  if (s[0] > s[1]) { uint16_t t = s[0]; s[0] = s[1]; s[1] = t; }
  if (s[1] > s[2]) { uint16_t t = s[1]; s[1] = s[2]; s[2] = t; }
  if (s[0] > s[1]) { uint16_t t = s[0]; s[0] = s[1]; s[1] = t; }
  return s[1];
}

/* ---------------- Range-Rate & Telemetry Helpers ---------------- */
static const char *get_state_label(radar_state_t st) {
  switch (st) {
    case STATE_SCAN:         return "SCAN ";
    case STATE_TARGET_LOCK:  return "LOCK ";
    case STATE_THREAT_ALERT: return "ALERT";
    default:                 return "ERR  ";
  }
}

static const char *get_motion_label(int16_t delta_cm) {
  if (delta_cm <= -3) { return "APPR"; }  /* Approaching target */
  if (delta_cm >=  3) { return "RECD"; }  /* Receding target    */
  return "HOLD";                          /* Hovering / static  */
}

static void update_display_and_uart(int16_t delta_cm) {
  char line0[17];
  char line1[17];

  snprintf(line0, sizeof(line0), "MODE:%s AZ:%3d", get_state_label(g_state), g_azimuth);

  if (g_curr_dist_cm >= 400U) {
    snprintf(line1, sizeof(line1), "RNG:--- cm CLEAR");
  } else if (g_state == STATE_SCAN) {
    snprintf(line1, sizeof(line1), "RNG:%3ucm  CLEAR", g_curr_dist_cm);
  } else {
    snprintf(line1, sizeof(line1), "RNG:%3ucm  %s ", g_curr_dist_cm, get_motion_label(delta_cm));
  }

  g_lcd.setCursor(0, 0);
  g_lcd.print(line0);
  g_lcd.setCursor(0, 1);
  g_lcd.print(line1);

  /* Structured CSV telemetry over UART (115200 baud) */
  char uart_buf[64];
  snprintf(uart_buf, sizeof(uart_buf), "AZ:%03d,RNG:%03u,DELTA:%+04d,STATE:%s",
           g_azimuth, g_curr_dist_cm, delta_cm, get_state_label(g_state));
  Serial.println(uart_buf);
}

/* ---------------- Setup ---------------- */
void setup(void) {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, HIGH);

  Serial.begin(115200);
  g_lcd.init();
  g_lcd.backlight();

  g_turret.attach(PIN_SERVO);
  g_turret.write(g_azimuth);

  Serial.println("=== Counter-Drone Radar Scanner & Target-Lock Controller Booted ===");
  Serial.println("FORMAT: AZ:<deg>,RNG:<cm>,DELTA:<cm>,STATE:<SCAN|LOCK|ALERT>");
}

/* ---------------- Non-Blocking Control Loop ---------------- */
void loop(void) {
  uint32_t now = millis();

  /* Non-blocking threat alarm oscillator in STATE_THREAT_ALERT */
  if (g_state == STATE_THREAT_ALERT) {
    if ((now - g_last_alarm_ms) >= ALARM_BLINK_MS) {
      g_last_alarm_ms = now;
      g_alarm_phase   = !g_alarm_phase;
      digitalWrite(PIN_LED_RED, g_alarm_phase ? HIGH : LOW);
      if (g_alarm_phase) {
        tone(PIN_BUZZER, 2200);
      } else {
        noTone(PIN_BUZZER);
      }
    }
  } else {
    noTone(PIN_BUZZER);
  }

  /* Select sampling interval based on active FSM mode */
  uint32_t period = (g_state == STATE_SCAN) ? SCAN_PERIOD_MS : TRACK_PERIOD_MS;
  if ((now - g_last_step_ms) < period) {
    return;
  }
  g_last_step_ms = now;

  /* Acquire median-filtered Time-of-Flight range */
  g_prev_dist_cm = g_curr_dist_cm;
  g_curr_dist_cm = read_filtered_distance_cm();

  int16_t delta_cm = 0;
  if ((g_curr_dist_cm < 400U) && (g_prev_dist_cm < 400U)) {
    delta_cm = (int16_t)g_curr_dist_cm - (int16_t)g_prev_dist_cm;
  }

  /* Deterministic 3-State FSM Execution */
  switch (g_state) {

    case STATE_SCAN:
      digitalWrite(PIN_LED_GREEN, HIGH);
      digitalWrite(PIN_LED_RED, LOW);

      if ((g_curr_dist_cm >= 2U) && (g_curr_dist_cm <= LOCK_RANGE_CM)) {
        g_miss_count = 0;
        g_state = (g_curr_dist_cm <= ALERT_ENTER_CM) ? STATE_THREAT_ALERT : STATE_TARGET_LOCK;
      } else {
        /* Advance azimuth sweep */
        g_azimuth += (int16_t)(g_sweep_dir * ANGLE_STEP_DEG);
        if (g_azimuth >= ANGLE_MAX_DEG) {
          g_azimuth   = ANGLE_MAX_DEG;
          g_sweep_dir = -1;
        } else if (g_azimuth <= ANGLE_MIN_DEG) {
          g_azimuth   = ANGLE_MIN_DEG;
          g_sweep_dir = 1;
        }
        g_turret.write(g_azimuth);
      }
      break;

    case STATE_TARGET_LOCK:
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, HIGH);

      if ((g_curr_dist_cm >= 2U) && (g_curr_dist_cm <= ALERT_ENTER_CM)) {
        g_miss_count    = 0;
        g_last_alarm_ms = now;
        g_state         = STATE_THREAT_ALERT;
      } else if ((g_curr_dist_cm >= 2U) && (g_curr_dist_cm <= LOCK_RANGE_CM)) {
        g_miss_count = 0;  /* Target still within lock zone */
      } else {
        g_miss_count++;
        if (g_miss_count >= MISS_LIMIT) {
          g_miss_count = 0;
          g_state      = STATE_SCAN;  /* Resume perimeter sweep */
        }
      }
      break;

    case STATE_THREAT_ALERT:
      digitalWrite(PIN_LED_GREEN, LOW);

      if ((g_curr_dist_cm >= 2U) && (g_curr_dist_cm <= ALERT_EXIT_CM)) {
        g_miss_count = 0;  /* Remain locked in high-threat alert */
      } else if ((g_curr_dist_cm > ALERT_EXIT_CM) && (g_curr_dist_cm <= LOCK_RANGE_CM)) {
        /* Hysteresis exit back to standard target lock */
        g_miss_count = 0;
        noTone(PIN_BUZZER);
        g_state = STATE_TARGET_LOCK;
      } else {
        g_miss_count++;
        if (g_miss_count >= MISS_LIMIT) {
          g_miss_count = 0;
          noTone(PIN_BUZZER);
          g_state = STATE_SCAN;
        }
      }
      break;
  }

  update_display_and_uart(delta_cm);
}
