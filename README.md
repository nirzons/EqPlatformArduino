# Equatorial Platform Stepper Controller (V14)

Arduino-based stepper motor controller firmware for **Nir's DIY Equatorial Platform for Latitude 32°**.

The platform is designed primarily for Dobsonian and alt-azimuth telescopes. By rotating the telescope around a polar axis parallel to Earth's rotational axis, it cancels out diurnal motion, keeping celestial targets centered in the eyepiece for over an hour at a time without manual nudging.

---

## Key Features

- **Geometry & Coverage:** Tailored for **latitude 32°**, offering excellent celestial tracking across **29° to 35°** latitude (North America, Mediterranean/Middle East, Asia, and adaptable to southern latitudes).
- **Tangent Arc Speed Compensation:** Automatically calculates and continuously adjusts motor RPM every 500 ms to compensate for geometry along the curved push arc.
- **Closed-Loop Adaptive Auto-Calibration:** Automatically measures undisturbed tracking runs from Start to Stop switch and auto-tunes travel distance in RAM in real time.
- **Precision Homing & Park:** Dual microswitches eliminate mechanical hysteresis on start (`homE`). Mid-session EEPROM Park (`PArk`) enables power-down and instant resume (`rESt`) in a Known Position state.
- **Hardware Protection:** Hardware interrupt-driven stepping (Timer1), full fault detection (`StSt`, `StSP`, `ovFd`, `ovRt`, `SwSh`, `dirE`), EEPROM fault logging, and acoustic warnings.
- **Minimalist Dark-Sky Interface:** 3-button control box (`Reset`, `Speed Up`, `Speed Down`), TM1637 4-digit display (countdown time, progress bar, percentage, mm), and audio buzzer feedback.

---

## 3D Models & Hardware Build Guide

All 3D models (STL files), hardware BOM, and assembly instructions are available on Thingiverse:  
👉 **Thingiverse Project:** [https://www.thingiverse.com/thing:6541079](https://www.thingiverse.com/thing:6541079)

---

## Documentation

Comprehensive field operating instructions, button references, calibration guides, and safety recovery protocols:  
📖 **[V14 User Manual (Markdown)](docs/USER_MANUAL.md)**

---

## Hardware Specifications

| Component | Specification |
| :--- | :--- |
| **Microcontroller** | Arduino Uno (ATmega328P, 16 MHz) |
| **Stepper Driver** | DM542 (1600 microsteps/rev, `STEP` / `DIR` / `ENABLE`) |
| **Stepper Motor** | NEMA 17 |
| **Drive Reduction** | 12T : 30T GT2 timing belt gears + T8 1 mm lead screw (0.4 mm linear travel/rev) |
| **Push Arc Radius** | 634.8 mm |
| **Default Travel Distance** | 170.5 mm (~61.4 min Sidereal tracking run) |
| **Display** | TM1637 4-Digit 7-Segment Red LED |
| **Audio** | Piezo Buzzer (Tone feedback, countdown alerts, alarms) |
| **Limit Switches** | Start & Stop mechanical microswitches (Active LOW, internal pullups) |

---

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
