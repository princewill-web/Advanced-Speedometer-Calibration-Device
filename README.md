# Advanced Speedometer Calibration Device (ASCD)

A low-cost speed measurement system built on an Arduino Uno. It calculates speed from the time an object takes to pass between two time-of-flight sensors placed a known distance apart. Graduation project (COM491), Near East University, supervised by Dr. Ramiz Musallam Salama.

## How it works
1. Two VL53L0X sensors are mounted a fixed, known distance apart.
2. A finite state machine (FSM) detects the object at sensor A, then sensor B, and ignores false triggers, partial passes and noise.
3. Speed is computed as **Speed = Distance / Time** and shown live on a 20×4 I2C LCD.

## Hardware
- Arduino Uno
- 2× VL53L0X time-of-flight sensors
- DS1302 real-time clock (RTC)
- 20×4 I2C LCD
- 2×AAA battery holder with switch

## Software
Arduino IDE (C++). Libraries: `Wire.h`, `LiquidCrystal_I2C`, `VL53L0X`, `DS1302`.

## Results
Stable, repeatable speed readings across multiple trials, with clear LCD output.

## Challenges and solutions
| Challenge | Solution |
|---|---|
| Sensor calibration sensitivity | FSM-based detection logic |
| Synchronisation delay between sensors | RTC integration for precise timing |
| Light and dust interference | Shielding and calibration recommendations |

## Applications
Educational labs, prototyping and robotics, research, and small-scale industrial monitoring (e.g. conveyor speed checks).

## Future work
Bluetooth/Wi-Fi remote monitoring, SD card or cloud data logging, and higher-precision calibration tools.

## Author
Princewill Ugochukwu Oguguom · [LinkedIn](PASTE-REAL-LINKEDIN-URL)

## License
MIT
