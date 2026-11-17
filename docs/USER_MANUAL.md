# Nir's Equatorial platform for latitude 32 - User Manual (V14)

---

## Introduction

An equatorial platform is an essential motorized observing aid designed primarily for Dobsonian and alt-azimuth telescopes. By rotating your telescope around a polar axis parallel to Earth's rotational axis, the platform cancels out Earth's rotation, keeping celestial objects centered in the eyepiece for over an hour at a time without manual nudging.

### What Makes This Platform Special
Designed and refined over several iterations, this platform is tailored specifically for **latitude 32°**, offering excellent celestial tracking across the latitude band of **29° to 35°**. This geographic range covers major stargazing hubs across the globe, including:
- **North America:** San Diego, Phoenix, Tucson, Dallas, San Antonio, Houston, Atlanta, Jacksonville
- **Mediterranean & Middle East:** Tel Aviv, Jerusalem, Cairo, Alexandria, Tripoli, Casablanca, Marrakesh
- **Asia:** Islamabad, Shanghai, Taipei, Fukuoka, Osaka, and many others.
*(With minor sector tilt adjustments, it can also be adapted for corresponding southern hemisphere latitudes such as Perth, Sydney, Santiago, and Buenos Aires.)*

### Construction & 3D Printing Recommendations
All 3D models, hardware lists, and physical assembly instructions are available on Thingiverse:
👉 **Thingiverse Project:** https://www.thingiverse.com/thing:6541079

- **Material:** High-temperature resistance material such as **PETG** (or ABS) is strongly recommended over standard PLA. Field stargazing equipment is frequently transported in warm car trunks and left under sunny skies during star parties, where PLA risks warping.
- **Infill & Resolution:** Print all structural parts with **20% to 25% infill**. The sector push arcs benefit from a fine layer height (**0.1 mm** or 0.2 mm) for a buttery-smooth contact surface, while the remaining brackets and mounts print excellently at **0.2 mm**. Arcs require a brim and supports during printing.

### Smart Firmware & Closed-Loop Adaptive Calibration
Under the hood, the V14 controller does much more than step a motor:
- **Tangent Arc Kinematics:** Automatically calculates and continuously adjusts motor RPM every 500 ms to compensate for tangential geometry changes along the curved push arc, ensuring drift-free tracking across the entire run.
- **Closed-Loop Adaptive Auto-Calibration:** As physical drive belts settle or seasonal temperatures shift, the firmware automatically measures every complete, undisturbed tracking run from Start to Stop switch. If actual travel varies from the nominal value, it auto-tunes its travel distance in RAM in real time, keeping countdown clocks, progress bars, and limit warnings spot-on without manual calibration sessions.
- **Precision Homing & Park:** Dual microswitches eliminate mechanical hysteresis, while an intelligent EEPROM Park feature lets you power down mid-session and resume observing later directly in a Known Position state.

---

## Table of Contents
1. [Overview & Hardware Specifications](#1-overview--hardware-specifications)
2. [Quick-Start Field Guide (Normal Observing Session)](#2-quick-start-field-guide-normal-observing-session)
3. [Complete Button Reference by Operating State](#3-complete-button-reference-by-operating-state)
4. [Precision Back-Off Homing (`homE`)](#4-precision-back-off-homing-home)
5. [Park & Persistent Resume (`PArk` / `OFF ` / `rESt`)](#5-park--persistent-resume-park--off---rest)
6. [Setup Menu (`Stup`) — All 5 Pages](#6-setup-menu-stup--all-5-pages)
7. [Platform Travel Calibration (3 Methods)](#7-platform-travel-calibration-3-methods)
8. [Display Mnemonics & Audio Signals Reference](#8-display-mnemonics--audio-signals-reference)
9. [Safety Monitor & Error Recovery (`Err `)](#9-safety-monitor--error-recovery-err-)
10. [Display Brightness & Night-Vision Filtering Tip](#10-display-brightness--night-vision-filtering-tip)

---

## 1. Overview & Hardware Specifications

### Platform Geometry & Drive Train
| Parameter | Value | Notes |
| :--- | :--- | :--- |
| **Controller Board** | **Arduino Uno board** (ATmega328P) | Controls stepper driver, display, buzzer, buttons, and limit switches |
| **Push Arc Radius** | `634.8 mm` | Tangent drive geometry calibrated for 32° latitude |
| **Default Travel Distance** | `170.5 mm` | Full sidereal run ~61.4 minutes (~15° platform sector arc) |
| **Allowable Calibration Bounds** | `153.5 – 187.5 mm` | ±10% sanity bounds (`0.9x` to `1.1x` default) |
| **Motor & Driver** | NEMA 17 + DM542 | `1600` microsteps/rev (`STEP` + `DIR` interface) |
| **Mechanical Reduction** | `12T : 30T` gears, T8 `1 mm` lead screw | `0.4 mm` linear carriage travel per motor revolution |
| **Tracking Rates** | **Sidereal** (`Side`), **Lunar** (`Luna`) | Optional **Solar** (`Sun`) via `#define SUN_RATE_SUPPORT` in `Config.h` |
| **Tracking Step Timing** | `150 µs` Timer1 ISR (`~185 steps/sec`) | Tangential cosine speed compensation every `500 ms` |
| **Fast Slew Step Timing** | `30 µs` Timer1 ISR (`-320 RPM` default) | Full rewind takes ~82 seconds at `-320 RPM` |

### Arduino Uno Pin Assignments
| Pin | Signal | Polarity / Type | Description |
| :--- | :--- | :--- | :--- |
| `D2` | Buzzer | `OUTPUT` (`tone()`) | Timer2 audio tones (countdown warnings, confirmations, alarms) |
| `D3` | `Reset` Button | `INPUT_PULLUP`, Active LOW | Master control button on control box |
| `D4` | Stepper `DIR` | `OUTPUT` | DM542 direction control |
| `D5` | Stepper `STEP` | `OUTPUT` | DM542 step pulses |
| `D6` | `Speed Down` Button | `INPUT_PULLUP`, Active LOW | Decelerate / Rewind / Down navigation |
| `D7` | `Speed Up` Button | `INPUT_PULLUP`, Active LOW | Accelerate / Fast-Forward / Up navigation |
| `D8` | Display `CLK` | `OUTPUT` | TM1637 4-digit display clock |
| `D9` | Display `DIO` | `BIDIR` | TM1637 4-digit display data |
| `D10` | Stepper `ENABLE` | `OUTPUT`, Active LOW | `LOW` = motor energized, `HIGH` = motor free/disabled |
| `D11` | `Stop` Microswitch | `INPUT_PULLUP`, Active LOW | End-of-travel physical limit switch |
| `D12` | `Start` Microswitch | `INPUT_PULLUP`, Active LOW | Mechanical Home physical limit switch |
| `D13` | Display `5V` | `OUTPUT` (`HIGH`) | Supplies 5V rail to TM1637 display |

### Control Box Layout
The control box includes:
- **3 Control Buttons (`Reset`, `Speed Up`, `Speed Down`):** All critical actions use `Reset` as the master button with intentional timing guard bands so short taps, rate toggles, setup entry, and chords cannot collide in the dark.
- **4-Digit Red 7-Segment Display (TM1637):** Shows 4-character status codes, remaining tracking time (`MM:SS`), progress bar, percentage, and travel distance in millimeters (`mm`).
- **Audio Buzzer:** Provides distinct tone patterns for button confirmations, tracking rate selection, `5:00` / `1:00` / `0:15` end-of-run countdown warnings, adaptive calibration chimes, and safety fault alerts.

---

## 2. Quick-Start Field Guide (Normal Observing Session)

- **Turn the platform on and press the `Reset` key to start tracking — that's it!**  
  Once started, the platform will continuously track (~61 minutes) and fast-reverse back to home (~82 seconds) automatically ("forever") until you pause or turn it off.

### Detailed Session Walkthrough
1. **Power On:**
   - **Normal startup:** The display powers on and shows **`rdy`** (Ready/Paused, motor disabled).
   - **If resuming from a saved Park:** Plays a double beep, flashes **`rESt`** for `1.2 s`, then shows **`rdy`**.
   - **If previous session ended with an unacknowledged fault:** Plays an ascending 3-tone alarm and flashes **`Err `** <-> fault code 3 times before settling at **`rdy`**.
2. **Start Tracking:**
   - **Short tap `Reset` to start tracking** (press and release cleanly in `< 0.5 s`).
   - **If sitting on the Start switch:** The platform first performs a brief **`homE`** back-off and re-approach sequence onto the switch before starting tracking (**`Strt`**).
3. **End-of-Run Warnings:**
   - `5:00` remaining: short `400 Hz` beep
   - `1:00` remaining: short `600 Hz` beep
   - `0:15` remaining: short `800 Hz` beep
4. **End of Travel (`Stop` Switch Reached):**
   - **By default (`Auto` mode):** Automatically reverses (**`rtrn`**) back to the Start switch and immediately resumes forward tracking (**`Strt`**).
   - **Optional `HAnd` (Manual) mode:** If selected in Setup (useful if you want to avoid a noisy, unexpected fast-return movement when a run ends), the platform stops and shows **`push`** until you press a speed button to rewind.
5. **Pause Anytime:**
   - Short tap `Reset` while tracking -> motor stops and disables, showing **`rdy`**.
   - From this **`rdy`** state, you can tap `Reset` again to resume tracking, or hold **`Speed Up`** / **`Speed Down`** (for `2.0 s`) to fast-forward (**`ff`**) or fast-reverse (**`rtrn`**) the platform.
6. **Powering Off Mid-Run (Optional Park):**
   - If you want to turn the platform off before a run ends and have it save its current position for your next session, you must **Park** it before switching off power.
   - While paused in **`rdy`**, hold **`Reset` + `Speed Down`** for `2.0 s` until you hear the chime and see **`PArk`** -> **`OFF `**, then switch off power (see [Section 5](#5-park--persistent-resume-park--off---rest)).

### Known Position vs. Unknown Position
At power-up, the platform is in an **Unknown Position** until either the **Start switch** or the **Stop switch** is triggered—at which point its exact position becomes **Known**. Resuming from **Park** (`rESt`) simply allows the system to power up directly in the **Known Position** state.

| Feature | Known Position | Unknown Position |
| :--- | :--- | :--- |
| **Tracking Clock (`MM:SS`)** | Counts **down** remaining time to `00:00` | Counts **up** elapsed time from `00:00` |
| **End-of-Run Warning Beeps** | Active (`5:00`, `1:00`, and `0:15` remaining) | Inactive (remaining distance to Stop switch is unknown) |
| **Fast Slew Display (`ff` / `rtrn`)** | Progress Bar, Remaining Time (`MM:SS`), or `%` | Elapsed time (`MM:SS`) only |
| **Arc Speed Compensation** | Exact position-adjusted RPM along the push arc | Nominal mid-arc tracking RPM |
| **Park (`PArk`)** | Allowed (saves exact remaining distance) | Rejected with **`noHm`** (cannot save unknown position) |

---

## 3. Complete Button Reference by Operating State

### Ready / Paused State (`"rdy"`)
*(Motor is disabled; `Speed Up` / `Speed Down` short taps are ignored to prevent accidental nudges.)*

| Gesture | Trigger Timing & Hold Window | Display / Audio Feedback | Action |
| :--- | :--- | :--- | :--- |
| **Tap `Reset`** | `30ms – 500ms` | Short beep -> `homE` or `Strt` | Starts/resumes tracking (runs `homE` only if currently on `Start` switch) |
| **Short hold `Reset`** | `1.5s – 3.0s` | `Side` / `Luna`<br>`800Hz / 500Hz` | Cycles active Tracking Rate (Sidereal / Lunar) |
| **Long hold `Reset`** | `>= 3.5s` | Double beep -> `Stup` | Enters the 5-page Setup Menu |
| **Hold `Speed Up`** | `2.0s` | Short beep -> `ff` | Starts Fast-Forward slew toward `Stop` switch |
| **Hold `Speed Down`** | `2.0s` | Short beep -> `rtrn` | Starts Fast-Return rewind toward `Start` switch |
| **Hold `Reset` + `Speed Down`** | `2.0s` | Chime -> `PArk` -> `OFF ` | Saves current homed position & rate to EEPROM and waits for power-off |

> **How Hold Timings Work & Safety Guard Bands:**
> - **Tap `Reset` & Short hold `Reset` (`1.5 s – 3.0 s`):** Evaluated **when you release** `Reset` (since the controller waits to see whether you are continuing to hold toward Setup).
> - **Long hold `Reset` (`>= 3.5 s`), `Speed Up`/`Down` (`2.0 s`), & Park (`2.0 s`):** Trigger **automatically while you are still holding the button(s)**—you do not need to time your release; just hold until you hear the beep or chime.
> - **Guard Deadbands:** Releasing `Reset` between `0.5 s – 1.5 s` or `3.0 s – 3.5 s` does nothing (prevents accidental triggers if you hold a tap too long or release a hold early).
> - **Chord Protection:** Pressing **`Reset` + `Speed Up`** or **`Speed Up` + `Speed Down`** is ignored. Releasing one button of a chord before `2.0 s` cancels the chord cleanly and mutes single-button `Reset` actions until `Reset` is released.

---

### Tracking State (`"Strt"` / Active Tracking)
| Gesture | Action |
| :--- | :--- |
| **Tap `Reset`** | Immediately stops motor, updates exact remaining travel distance, and returns to **`rdy`** |
| **Press / Hold `Speed Up`** | Increases angular velocity by `+1 µrad/min`. Displays offset in `µrad/min` for `2 s`. |
| **Press / Hold `Speed Down`** | Decreases angular velocity by `-1 µrad/min`. Displays offset in `µrad/min` for `2 s`. |

*(Note: Making a manual speed adjustment during tracking disqualifies that run from Adaptive Auto-Calibration.)*

---

### Fast-Forward (`"ff"`) & Fast-Return (`"rtrn"`) States
| Gesture | Action |
| :--- | :--- |
| **Tap `Reset`** | Immediately halts slew, updates remaining forward/return travel coordinates cleanly, and returns to **`rdy`** |
| **Tap `Speed Up` or `Speed Down`** *(after `2 s` initial label)* | Cycles progress view: **Progress Bar** -> **Time Remaining (`MM:SS`)** -> **Percentage (`XX%`)** |

> **Progress Bar and Percentage Display:**
> - Formed by vertical bars filling from left to right (`|` -> `||` -> `|||` ...) or percentage digits indicating carriage position along the track.
> - **Full bar / 100%:** Carriage is at the **Stop switch** (end of travel).
> - **Empty bar / 0%:** Carriage is at the **Start switch** (beginning of travel).
> - *(In Unknown Position, elapsed time `MM:SS` is shown instead.)*

---

### Waiting for Manual Return State (`"push"`)
*(Entered only when tracking reaches the Stop switch in `HAnd` mode, or when Fast-Forward reaches the Stop switch.)*

| Gesture | Action |
| :--- | :--- |
| **Tap `Speed Up` or `Speed Down`** | Begins Fast-Return (`rtrn`) back to the Start switch (and immediately resumes tracking upon arrival) |
| **Tap `Reset`** | Exits to **`rdy`** (retaining homed overrun state) |

---

## 4. Precision Back-Off Homing (`homE`)

Microswitches have small mechanical hysteresis between the point they press (`LOW`) and release (`HIGH`). If the platform powers on already resting on the Start switch, it cannot know how deep into the switch arm it is sitting.

**When it runs:** **Only** when starting tracking from **`rdy`** while the Start limit switch is already pressed. *(Normal `Auto` or `Manual` returns that hit the Start switch from outside during Fast-Return (`rtrn`) already catch the exact switch click and start tracking immediately without backing off.)*

**3-Phase Sequence:**
1. **Clear Forward (`160 RPM`, ~1.1 mm/s):** Drives forward until the Start switch releases (`HIGH`), then moves an extra `0.8 mm` for positive leaf clearance (capped at `6.0 mm` safety limit -> `StSt` fault if stuck).
2. **Settle (`100 ms`):** Brief dwell with motor paused.
3. **Creep Reverse (`-160 RPM`, ~1.1 mm/s):** Reverses back until the exact instant the Start switch clicks `LOW` (capped at forward travel `+ 2.0 mm` -> `StSt` fault if switch fails to close).
4. **Lock & Track:** Sets remaining distance to the full calibrated travel distance, marks the platform **Homed**, emits a short beep, and immediately starts tracking (`Strt`).

- Pressing **`Reset`** at any point during `homE` aborts homing and returns to **`rdy`**.

---

## 5. Park & Persistent Resume (`PArk` / `OFF ` / `rESt`)

Use **Park** if you want to turn off power mid-run and have the platform power up later directly in a **Known Position**, resuming from the exact same spot without needing to return to Start.

### Prerequisite
The platform must be in a **Known Position** (i.e. started from Start/Stop in this session or resumed from a prior Park). Attempting to park from an **Unknown Position** flashes **`noHm`** with a low `300 Hz` reject beep and stays in **`rdy`**.

### Step-by-Step Park Procedure
1. **Pause to `"rdy"`:** If tracking, short-tap **`Reset`** and release it completely.
2. **Hold `Reset` + `Speed Down` together for `2.0 s`:**
   - The `2.0 s` timer runs once both buttons are held simultaneously.
   - *(If you release `Speed Down` before `2.0 s`, Park is cleanly cancelled and nothing triggers when you release `Reset`.)*
3. **Wait for Confirmation:**
   - At `2.0 s`, the position and tracking rate are saved to EEPROM.
   - Display shows **`PArk`** with an ascending chime (`600 Hz` -> `900 Hz`).
   - `1 second` later, the display switches to **`OFF `**.
4. **Power Off (or Unpark):**
   - Release both buttons and turn off the main power switch while **`OFF `** is displayed.
   - **To cancel Park without powering off:** While **`OFF `** is shown, press **`Reset`** (~`40 ms`). This safely clears the EEPROM park record, keeps your position in RAM, and returns to **`rdy`**.

### Next Power-On Behavior
Restores the exact saved position and tracking rate, plays a double beep, flashes **`rESt`** for `1.2 s`, and enters **`rdy`** in the **Known Position** state. Short tap **`Reset`** to resume tracking.

---

## 6. Setup Menu (`Stup`) — All 5 Pages

- **Enter Setup:** In **`rdy`**, hold **`Reset`** for **`>= 3.5 s`** until you hear a double beep and see **`Stup`**. Release `Reset`.
- **Navigate Pages:** Short-tap **`Reset`** (`30 ms – 1500 ms`) to advance through the 5 pages in order.
- **Auto-Exit & Save:** After **`8.0 seconds`** of no button activity, Setup displays a numeric countdown during the final **`3.0 seconds`**, saves any travel distance change to EEPROM (only if modified), updates location RPM, and exits to **`rdy`**. Any button press resets the `8.0 s` idle timer.

### Setup Page Reference

| Page | Function | Display Label | Controls & Actions |
| :--- | :--- | :--- | :--- |
| **1** | **Return Policy & Speed** | `Stup` | • **`Speed Up`**: Cycles Policy: `Auto` -> `HAnd` -> `test`<br>• **`Speed Down`**: Cycles Return Speed: `-320` -> `-240` -> `-160` -> `-80` RPM (bounces back through `-160` -> `-240` -> `-320`) |
| **2** | **Manual Travel Distance** | `dist` (shows `XXX.X mm` after 1s or on speed press) | • **`Speed Up`**: Increases travel by `+0.1 mm`<br>• **`Speed Down`**: Decreases travel by `-0.1 mm`<br>*(Auto-saved to EEPROM on 8s exit or FCAL start)* |
| **3** | **Fault History Log** | `nonE` / Fault | • **`Speed Down`**: Clears stored fault history to `nonE` |
| **4** | **Adaptive Distance Preview** | `Auto` -> `XXX.X` | • **Hold `Reset` (`2.0 s`)**: Saves auto-tuned RAM distance to EEPROM (`SAVd`) |
| **5** | **Fast Calibration** | `FCAL` | • **`Speed Up` or `Speed Down`**: Launches Fast Calibration run toward Stop switch |

*(Notes: On Page 1, simultaneous Speed Up + Down does not change settings. On Page 2, simultaneous Speed Up + Down is ignored. Return policy and return speed changes are active in RAM for the current power session.)*

---

## 7. Platform Travel Calibration (3 Methods)

Accurate total travel calibration ensures the `MM:SS` countdown reaches `00:00` at the exact instant the Stop switch clicks and keeps end-of-travel kinematics aligned.

### Method 1: Fast Calibration (`FCAL`, ~5.5 minutes)
Ideal right after building a platform or adjusting a microswitch position:
1. Move the platform back onto the **Start** limit switch (e.g., via Fast-Return).
2. Enter Setup (**Hold `Reset` 3.5 s**) and tap **`Reset`** 4 times to reach **`FCAL`**.
3. Press **`Speed Up` or `Speed Down`**. *(If not touching the Start switch, it flashes **`noHm`** and refuses to start.)*
4. Display shows **`CAL `**, then updates traveled **`mm`** every `1.5 s` as it moves forward at `80 RPM` (`~0.52 mm/s`).
   *(Tap `Reset` anytime to abort back to `rdy`.)*
5. Upon reaching the Stop switch, the motor stops and alternates **`diSt`** <-> **`XXX.X`** every `1.0 s`.
   - **To Save to EEPROM:** Hold **`Reset` for `2.0 s`** -> flashes **`SAVd`** with save chime and returns to **`rdy`**. *(Values outside `153.5–187.5 mm` are rejected with a low beep.)*
   - **To Discard:** Short-tap **`Reset`** -> exits to **`rdy`** without modifying EEPROM.

### Method 2: Slow Tracking Calibration (`test` mode, ~61 minutes)
Measures end-to-end travel under real tracking load at Sidereal speed:
1. In Setup Page 1 (`Stup`), press `Speed Up` twice until **`test`** appears, and wait `8 s` to exit to **`rdy`**.
2. Start from the Start switch by tapping **`Reset`** (let **`homE`** complete).
3. While tracking, the display shows **live traveled millimeters** instead of `MM:SS`.
4. When the Stop switch trips, the platform enters the same **`diSt`** <-> **`XXX.X`** review screen as Fast Calibration: hold **`Reset` `2.0 s`** to save (`SAVd`) or tap `Reset` to discard.

### Method 3: Closed-Loop Adaptive Auto-Calibration
Automatically fine-tunes the active travel distance immediately in RAM during normal observing nights:
- **Eligibility ("Clean Full Run"):** Only evaluates runs that:
  1. Started from a full home (`homE` or Start-switch arrival from Fast-Return),
  2. Ran completely to the Stop switch without any user pause, fast-forward, rewind, or fault,
  3. Remained at **Sidereal** rate with zero manual speed adjustments (`0 µrad/min`).
- **Immediate RAM Adaptation & Filtering:** Ignores small errors `< 0.1 mm` (deadband) and rejects outliers `> 5.0 mm` (e.g., if bumped). Applies a `33%` low-pass filter step directly to the active travel distance in RAM (clamped within `153.5–187.5 mm`) so the very next run immediately uses the improved travel distance.
- **End-of-Run Notifications (Before Pausing):**
  - Plays a **triple ascending chime** (`600` -> `800` -> `1000 Hz`) the moment the Stop switch triggers and an adjustment occurs.
  - During the **first 10 seconds of `Auto` return (`rtrn`)**, shows `"rtrn"` for `2 s` and then alternates **`CAL `** <-> **`XXX.X`** (the new live distance in mm) every `1.0 s` before switching to the standard progress view (press `Speed Up` or `Speed Down` to skip straight to progress view).
  - In **`HAnd` mode (`push`)**, alternates **`push`** <-> **`CAL `** every `1.5 s`.
- **Save Reminder (`CAL ` <-> `rdy`):** If the live RAM distance drifts **`>= 0.5 mm`** (~11 s of tracking) from the saved EEPROM value, **`rdy`** also alternates with **`CAL `** every `1.5 s` when paused (and the `10 s` return banner continues to appear on subsequent returns until saved or cleared).
- **Making it Permanent:** Enter Setup Page 4 (**`Auto`**), confirm the displayed `XXX.X` value, and hold **`Reset` for `2.0 s`** until **`SAVd`** confirms the EEPROM write.

---

## 8. Display Mnemonics & Audio Signals Reference

### 4-Character Display Mnemonics
| Display | Meaning |
| :--- | :--- |
| `rdy` | Ready / Paused (motor disabled) |
| `Strt` | Starting tracking (`2 s` banner) |
| `homE` | Running 3-phase precision back-off homing |
| `ff` | Fast-Forward slewing (`2 s` banner before bar/time/%) |
| `rtrn` | Fast-Return rewinding (`2 s` banner before bar/time/%) |
| `push` | End of travel reached (`HAnd` mode); waiting to rewind |
| `Side` / `Luna` / `Sun` | Active tracking rate (`Sun` only if compiled with `SUN_RATE_SUPPORT`) |
| `PArk` -> `OFF ` | Position & rate saved to EEPROM; safe to power off (or `Reset` to unpark) |
| `rESt` | Restored parked position from EEPROM on boot |
| `noHm` | Action rejected — platform must be homed (or sitting on Start switch for `FCAL`) |
| `Stup` | Setup Page 1 — Return Policy (`Auto`/`HAnd`/`test`) & Return Speed (`-320`..`-80`) |
| `dist` | Setup Page 2 — Manual Travel Distance (`±0.1 mm`) |
| `nonE` | Setup Page 3 — No fault recorded (or Fault History cleared) |
| `Auto` | Setup Page 4 (Adaptive distance preview) OR Page 1 `AUTO_RT` policy |
| `HAnd` | Page 1 `MANUAL_RT` policy |
| `test` | Page 1 `MEASURE_RT` slow calibration policy |
| `FCAL` | Setup Page 5 — Fast Calibration launcher |
| `CAL ` | Fast Calibration running, Save alert (`CAL ` <-> `rdy` / `push`), or 10s return banner (`CAL ` <-> `XXX.X`) |
| `diSt` <-> `XXX.X` | Calibration complete — displays measured distance (`2 s Reset` saves, `Tap` exits) |
| `SAVd` | Calibrated travel distance saved to EEPROM |
| `Err ` <-> `XXXX` | Fault active (or boot unacknowledged fault alert) — see Section 9 |

### Audio Signal Reference
| Pattern | Frequencies / Timing | Meaning |
| :--- | :--- | :--- |
| **Single short beep** | `400 Hz`, `40 ms` | Tracking start/slew start, homing complete, calibration complete, fault recovered, fault cleared |
| **Double beep** | `600 Hz` `50 ms` x2 (`80 ms` gap) | Entered Setup (`Stup`), OR restored Park on boot (`rESt`) |
| **Rate tone** | `800 Hz` (`Side`), `500 Hz` (`Luna`), `650 Hz` (`Sun`) | Confirms active tracking rate when toggled |
| **Park chime** | `600 Hz` `80 ms` -> `900 Hz` `150 ms` | Park position safely written to EEPROM (`PArk`) |
| **Save chime** | `700 Hz` `80 ms` -> `1000 Hz` `120 ms` | Calibration distance written to EEPROM (`SAVd`) |
| **Adaptive triple chime** | `600 Hz` -> `800 Hz` -> `1000 Hz` | Active travel distance auto-tuned in RAM at end of clean run |
| **Countdown alerts** | `400 Hz` (`5 min`), `600 Hz` (`1 min`), `800 Hz` (`15 sec`) | Remaining tracking time alerts (`40 ms` each) |
| **Reject beep** | `300 Hz`, `200 ms` | Invalid action (`noHm`, boundary limit hit, out-of-range calibration, shorted recovery) |
| **Error alarm** | `500 Hz` `100 ms` x3; then `40 ms` chirp every `30 s` | Safety fault entered (speed button mutes 30s chirp) |
| **Boot fault alarm** | `400 Hz` -> `600 Hz` -> `800 Hz` | Unacknowledged fault from previous session |

---

## 9. Safety Monitor & Error Recovery (`Err `)

When any safety guard triggers, the firmware **immediately disables the stepper motor**, **stops Timer1**, logs the fault to EEPROM (`unacknowledged`), sounds the 3-beep alarm, and alternates **`Err `** with the 4-character mnemonic every `1.0 s`.

| Mnemonic | Fault Name | Likely Cause & Trigger Condition |
| :--- | :--- | :--- |
| `StSt` | Start Switch Stuck | Start switch remained active after `> 2.7 mm` forward travel, OR failed to clear/re-close during `homE` (`> 6.0 mm` forward / `> clear + 2.0 mm` reverse). |
| `StSP` | Stop Switch Stuck | Stop switch remained active after `> 2.7 mm` of return travel. |
| `ovFd` | Overtravel Forward | Carriage passed `totalTravelDistanceMm + 2.7 mm` (tracking) or `+ 3.5 mm` (`ff` / post-home overrun) without hitting Stop switch. |
| `ovRt` | Overtravel Return | Carriage passed `totalTravelDistanceMm + 3.5 mm` (or `+ 3.5 mm` past virtual zero) without hitting Start switch. |
| `SwSh` | Limit Switch Short | Both Start and Stop switches read `LOW` simultaneously for `4` consecutive samples (wiring short or broken NC/NO common). |
| `dirE` | Wrong Switch Direction | Departure switch re-activated after already clearing `> 5.4 mm` away (`2 * 2.7 mm`), indicating reversed motor direction or swapped limit switch plugs. |

### Boundary Guard Interlocks (Non-Error Rejections)
In addition to the critical safety faults above, the controller enforces soft boundary protection while paused in **`rdy`**:
- **Slew Boundary Rejection:** If the carriage is already physically resting on the **Start switch**, attempting to start Fast-Return (**Hold `Speed Down`**) is safely ignored and rejected with a low `300 Hz` beep. Likewise, if the carriage is resting on the **Stop switch**, attempting to start Fast-Forward (**Hold `Speed Up`**) is rejected with the same `300 Hz` tone. This prevents driving the mechanical carriage deeper into a physical end-stop.

### Clearing & Recovering from an Error (`"Err "`)
1. **Mute the 30-second reminder chirp (optional):** Press **`Speed Up`** or **`Speed Down`**.
2. **Inspect the platform hardware:** Ensure switches are not stuck/shorted and mechanical travel is clear.
3. **Hold `Reset` for `2.0 seconds`:** Marks the fault record as acknowledged in EEPROM, clears the fault, resets position to unhomed (`rdy`), and emits a confirmation beep.
   - *(Note: If both switches are still physically shorted together, recovery is rejected with a `300 Hz` beep to protect EEPROM and mechanics until the short is cleared.)*
4. **Inspect or Clear Past Faults Later:** Open Setup (**Hold `Reset` 3.5 s**) and tap twice to Page 3 to view the last recorded fault mnemonic; press **`Speed Down`** to clear it to **`nonE`**.

---

## 10. Display Brightness & Night-Vision Filtering Tip

Although the firmware configures the TM1637 4-digit red display to its lowest hardware brightness setting (`0`), the bare LEDs are still quite bright under dark skies.

> [!TIP]
> **Field Filtering Trick:**
> A simple and effective practical solution is to **tape a piece of black nylon (cut from a black trash bag) over the display**. In the dark, the digits remain crisp and easy to read through the nylon while filtering the brightness down to a comfortable, dark-adaptation-friendly level.

