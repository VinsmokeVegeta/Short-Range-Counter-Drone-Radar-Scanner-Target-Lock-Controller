# Short-Range Counter-Drone Radar Scanner & Target-Lock Controller

A real-time target detection, azimuth lock-on, and perimeter threat-alert controller built on Arduino Uno (ATmega328P) in Embedded C style. Combines a servo-driven Time-of-Flight (ToF) ultrasonic scanner with a 3-state tracking Finite State Machine (`SCAN` -> `TARGET_LOCK` -> `THREAT_ALERT`), 3-sample median echo filtering, hysteresis thresholding, and live UART telemetry.

## Hardware Architecture & Pinout

| Peripheral | Arduino Uno Pin | Interface / Role |
| :--- | :--- | :--- |
| HC-SR04 Ultrasonic Sensor | `D7` (TRIG), `D6` (ECHO) | Pulsed Time-of-Flight rangefinder (`2–340 cm`) |
| Servo Turret Actuator | `D9` (Timer1 PWM) | Azimuth sweep (`15°–165°`) and closed-loop target lock |
| 16x2 I2C LCD (`0x27`) | `A4` (SDA), `A5` (SCL) | Live mode, azimuth angle, range, and motion vector (`APPR`/`RECD`/`HOLD`) |
| Status LEDs + Piezo Buzzer | `D4` (Red), `D5` (Green), `D3` (Timer2) | Visual scan/lock status and pulsing high-threat alarm |

## Key Control & Embedded Concepts
- **3-State Tracking FSM:** Automatically transitions from perimeter sweep (`SCAN`) to azimuth hold (`TARGET_LOCK`, $\le 120\text{ cm}$) and high-threat alarm (`THREAT_ALERT`, $\le 50\text{ cm}$).
- **Noise & Chattering Immunity:** Applies a 3-sample median filter to reject ultrasonic spikes, a $5\text{ cm}$ hysteresis deadband (`50 cm` enter / `55 cm` exit) at the alert boundary, and a 3-miss confirmation filter before releasing target lock.
- **Hardware Timer Partitioning:** Separates `Timer0` (`millis()`), `Timer1` (`Servo` PWM), and `Timer2` (buzzer `tone()`) for jitter-free non-blocking execution.
- **Live Telemetry:** Streams structured CSV logs (`AZ:<deg>,RNG:<cm>,DELTA:<cm>,STATE:<mode>`) over UART at `115200` baud.

## Live Simulation
- **Wokwi Project:** [Run Live in Browser](https://wokwi.com/projects/481397933644866561)
- **Demo Screenshot:**
  ![Counter-Drone Radar Demo](simulation_proof)
