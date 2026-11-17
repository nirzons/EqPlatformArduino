# Equatorial Platform Stepper Controller — User Manual (v14)

**Target Platform:** Custom Equatorial Platform for Latitude 32° (`PLATFORM_NIR`, ATmega328P / Arduino Uno)  
**Controller:** 3-Button Hand Control + 4-Digit Red 7-Segment Display + Audio Buzzer  

---

## Table of Contents
1. [Overview & Hardware Specifications](#1-overview--hardware-specifications)
2. [Quick-Start Field Guide (Normal Observing Session)](#2-quick-start-field-guide-normal-observing-session)
3. [Complete Button Reference by Operating State](#3-complete-button-reference-by-operating-state)
4. [Precision Back-Off Homing (`homE`)](#4-precision-back-off-homing-home)
5. [Park & Persistent Resume (`PArk` -> `OFF ` -> `rESt`)](#5-park--persistent-resume-park---off----rest)
6. [Setup Menu (`Stup`) — All 5 Pages](#6-setup-menu-stup--all-5-pages)
7. [Platform Travel Calibration (3 Methods)](#7-platform-travel-calibration-3-methods)
8. [Display Mnemonics & Audio Signals Reference](#8-display-mnemonics--audio-signals-reference)
9. [Safety Monitor & Error Recovery (`Err `)](#9-safety-monitor--error-recovery-err-)

---

## 1. Overview & Hardware Specifications

### Platform Geometry & Drive Train
| Parameter | Value | Notes |
| :--- | :--- | :--- |
| **Push Arch Radius** | `634.8 mm` | Tangent drive geometry calibrated for 32° latitude |
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
| `D3` | `Reset` Button | `INPUT_PULLUP`, Active LOW | Master control button on hand controller |
| `D4` | Stepper `DIR` | `OUTPUT` | DM542 direction control |
| `D5` | Stepper `STEP` | `OUTPUT` | DM542 step pulses |
| `D6` | `Speed Down` Button | `INPUT_PULLUP`, Active LOW | Decelerate / Rewind / Down navigation |
| `D7` | `Speed Up` Button | `INPUT_PULLUP`, Active LOW | Accelerate / Fast-Forward / Up navigation |
| `D8` | Display `CLK` | `OUTPUT` | TM1637 4-digit display clock |
| `D9` | Display `DIO` | `BIDIR` | TM1637 4-digit display data (default brightness `0` = dimmest) |
| `D10` | Stepper `ENABLE` | `OUTPUT`, Active LOW | `LOW` = motor energized, `HIGH` = motor free/disabled |
| `D11` | `Stop` Microswitch | `INPUT_PULLUP`, Active LOW | End-of-travel physical limit switch |
| `D12` | `Start` Microswitch | `INPUT_PULLUP`, Active LOW | Mechanical Home physical limit switch |
| `D13` | Display `5V` | `OUTPUT` (`HIGH`) | Supplies 5V rail to TM1637 display |

### Hand Controller Layout
The hand controller has **3 buttons** (`Reset`, `Speed Up`, `Speed Down`). All critical actions use `Reset` as the master button with intentional guard bands so short taps, rate toggles, setup entry, and chords cannot collide in the dark.

---

## 2. Quick-Start Field Guide (Normal Observing Session)

1. **Power On:**
   - The display powers on at dimmest brightness (`0`) to protect dark adaptation.
   - **Normal startup:** Shows **`rdy`** (Ready/Paused, motor disabled).
   - **If resuming from a saved Park:** Plays a double beep, flashes **`rESt`** for `1.2 s`, then shows **`rdy`**.
   - **If previous session ended with an unacknowledged fault:** Plays an ascending 3-tone alarm and flashes **`Err `** <-> fault code 3 times before settling at **`rdy`**.
2. **Start Tracking:**
   - **Short tap `Reset`** (press and release cleanly in `< 0.5 s`).
   - If sitting on the Start switch, the platform runs a brief ~3–4 second **`homE`** precision back-off sequence, beeps, shows **`Strt`** for `2 s`, then displays the remaining time (`MM:SS`, or traveled `mm` if in Measure mode).
   - If already mid-platform (paused or unparked), it skips homing and immediately starts tracking (`Strt`).
3. **End-of-Run Warnings:**
   - `5:00` remaining: short `400 Hz` beep
   - `1:00` remaining: short `600 Hz` beep
   - `0:15` remaining: short `800 Hz` beep
4. **End of Travel (`Stop` Switch Reached):**
   - **In `Auto` mode (default):** Automatically reverses to **`rtrn`** (`-320 RPM` default), shows progress (`Bar` -> `MM:SS` -> `%`), hits the Start switch, resets remaining time to full (`170.5 mm` calibrated), and **immediately resumes forward tracking (`Strt`)**.
   - **In `HAnd` (Manual) mode:** Stops motor and displays **`push`**. Tap `Speed Up` or `Speed Down` when ready to rewind (and auto-resume at Start), or tap `Reset` to return to **`rdy`**.
5. **Pause Anytime:**
   - Short tap `Reset` while tracking -> motor stops and disables, showing **`rdy`**.
6. **Powering Off Mid-Run (Optional Park):**
   - While paused in **`rdy`**, hold **`Reset` + `Speed Down`** for `2.0 s` until you hear the chime and see **`PArk`** -> **`OFF `**, then switch off power (see [Section 5](#5-park--persistent-resume-park---off----rest)).

---

## 3. Complete Button Reference by Operating State

### Ready / Paused State (`"rdy"` — `STOP_ST`)
*(Motor is disabled; `Speed Up` / `Speed Down` short taps are ignored to prevent accidental nudges.)*

| Gesture | Trigger Timing | Display / Audio Feedback | Action |
| :--- | :--- | :--- | :--- |
| **Tap `Reset`** | `30 ms – 500 ms` (on release) | Short beep -> `homE` or `Strt` | Starts/resumes tracking (runs `homE` only if currently on `Start` switch) |
| **Hold `Reset`** | `1.5 s – 3.0 s` (on release) | `Side` (`800 Hz`) / `Luna` (`500 Hz`) / `Sun` (`650 Hz`) | Cycles active Tracking Rate |
| **Hold `Reset`** | `>= 3.5 s` (triggers while held) | Double beep -> `Stup` | Enters the 5-page Setup Menu |
| **Hold `Speed Up`** | `2.0 s` (triggers while held) | Short beep -> `ff` | Starts Fast-Forward slew toward `Stop` switch |
| **Hold `Speed Down`** | `2.0 s` (triggers while held) | Short beep -> `rtrn` | Starts Fast-Return rewind toward `Start` switch (rejected with `300 Hz` beep if already on `Start` switch) |
| **Hold `Reset` + `Speed Down`** | `2.0 s` (triggers while held) | Chime -> `PArk` -> `OFF ` | Saves current homed position & rate to EEPROM and waits for power-off |

> **Deadbands & Chord Safety:**
> - Releasing `Reset` between `0.5 s` and `1.5 s`, or between `3.0 s` and `3.5 s`, does nothing (prevents accidental triggers if you hold slightly too long or release slightly early).
> - Pressing **`Reset` + `Speed Up`** or **`Speed Up` + `Speed Down`** simultaneously in `rdy` is safely ignored.
> - Pressing both buttons for a chord and releasing one early before `2.0 s` cancels the chord cleanly and mutes single-button `Reset` actions until `Reset` is released.

---

### Tracking State (`RUNNING_ST`)
| Gesture | Action |
| :--- | :--- |
| **Tap `Reset`** | Immediately stops motor, updates exact remaining travel distance, and returns to **`rdy`** |
| **Press / Hold `Speed Up`** | Increases angular velocity by `+1 µrad/min` (repeats every `200 ms`; clamped to `±50%` of Sidereal rate). Displays offset in `µrad/min` for `2 s`. |
| **Press / Hold `Speed Down`** | Decreases angular velocity by `-1 µrad/min` (repeats every `200 ms`). Displays offset in `µrad/min` for `2 s`. |

*(Note: Making a manual speed adjustment during tracking disqualifies that run from Adaptive Auto-Calibration.)*

---

### Fast-Forward (`"ff"`) & Fast-Return (`"rtrn"`) States
| Gesture | Action |
| :--- | :--- |
| **Tap `Reset`** | Immediately halts slew, updates remaining forward/return travel coordinates cleanly, and returns to **`rdy`** |
| **Tap `Speed Up` or `Speed Down`** *(after `2 s` initial label)* | Cycles progress view: **Progress Bar** -> **Time Remaining (`MM:SS`)** -> **Percentage (`XX%`)** |

---

### Waiting for Manual Return State (`"push"` — `WAIT_RETURN_ST`)
| Gesture | Action |
| :--- | :--- |
| **Tap `Speed Up` or `Speed Down`** | Begins Fast-Return (`rtrn`) back to the Start switch (and immediately resumes tracking upon arrival) |
| **Tap `Reset`** | Exits to **`rdy`** (retaining homed overrun state) |

---

## 4. Precision Back-Off Homing (`homE`)

Microswitches have small mechanical hysteresis between the point they press (`LOW`) and release (`HIGH`). If the platform powers on already resting on the Start switch, it cannot know how deep into the switch arm it is sitting.

**When it runs:** **Only** when starting tracking from **`rdy`** while the Start limit switch is already pressed. *(Normal `Auto` or `Manual` returns that hit the Start switch from outside during `RETURN_ST` already catch the exact switch click and start tracking immediately without backing off.)*

**3-Phase Sequence:**
1. **Clear Forward (`160 RPM`, ~1.1 mm/s):** Drives forward until the Start switch releases (`HIGH`), then moves an extra `0.8 mm` for positive leaf clearance (capped at `6.0 mm` safety limit -> `StSt` fault if stuck).
2. **Settle (`100 ms`):** Brief dwell with motor paused.
3. **Creep Reverse (`-160 RPM`, ~1.1 mm/s):** Reverses back until the exact instant the Start switch clicks `LOW` (capped at forward travel `+ 2.0 mm` -> `StSt` fault if switch fails to close).
4. **Lock & Track:** Sets `remainingDistanceMm_ = totalTravelDistanceMm_`, marks the platform **Homed**, emits a short beep, and immediately starts tracking (`Strt`).

- Pressing **`Reset`** at any point during `homE` aborts homing and returns to **`rdy`**.

---

## 5. Park & Persistent Resume (`PArk` -> `OFF ` -> `rESt`)

Use **Park** if you want to turn off controller power mid-observing run and resume from the exact same sky position later without re-homing from Start.

### Prerequisite
The platform must be **Homed** (`isHomed_ == true` with `remainingDistanceMm_ > 0` — i.e., started from Start in this session or resumed from a prior Park). Trying to park while unhomed flashes **`noHm`** with a low `300 Hz` reject beep and stays in **`rdy`**.

### Step-by-Step Park Procedure
1. **Pause to `"rdy"`:** If tracking, short-tap **`Reset`** and release it completely.
2. **Hold `Reset` + `Speed Down` together for `2.0 s`:**
   - The `2.0 s` timer runs once both buttons are held simultaneously.
   - *(If you release `Speed Down` before `2.0 s`, Park is cleanly cancelled and nothing triggers when you release `Reset`.)*
3. **Wait for Confirmation:**
   - At `2.0 s`, the EEPROM Park record (`EP_ADDR_PARK_RECORD`) is written with `remainingDistanceMm_`, `TrackingRate`, magic `0x4551`, and checksum.
   - Display shows **`PArk`** with an ascending chime (`600 Hz` -> `900 Hz`).
   - `1 second` later, the display switches to **`OFF `**.
4. **Power Off (or Unpark):**
   - Release both buttons and turn off the main power switch while **`OFF `** is displayed.
   - *(Note: Even if you hold `Reset` longer after **`OFF `** appears, a `25 ms` release guard ensures it stays parked until you actually release `Reset`.)*
   - **To cancel Park without powering off:** While **`OFF `** is shown, press **`Reset`** (~`40 ms`). This invalidates the EEPROM park record (preventing stale recovery later), keeps your position in RAM, and returns to **`rdy`**.

### Next Power-On Behavior
- **If the carriage is NOT resting on the Start switch:** Restores `remainingDistanceMm_` and `TrackingRate`, **immediately clears the EEPROM park signature** (fail-safe against power cuts during subsequent movement), plays a double beep, flashes **`rESt`** for `1.2 s`, and enters **`rdy`**. Tap **`Reset`** to resume tracking.
- **If the Start switch IS physically pressed on boot:** Physical home takes priority — the stale EEPROM park record is automatically cleared and the platform boots normally to **`rdy`**.

---

## 6. Setup Menu (`Stup`) — All 5 Pages

- **Enter Setup:** In **`rdy`**, hold **`Reset`** for **`>= 3.5 s`** until you hear a double beep and see **`Stup`**. Release `Reset`.
- **Navigate Pages:** Short-tap **`Reset`** (`30 ms – 1500 ms`) to advance through the 5 pages in order.
- **Auto-Exit & Save:** After **`8.0 seconds`** of no button activity, Setup displays a numeric countdown during the final **`3.0 seconds`**, saves any travel distance change to EEPROM (only if modified), updates location RPM, and exits to **`rdy`**. Any button press resets the `8.0 s` idle timer.

| Page | Display Label | `Speed Up` Action | `Speed Down` Action | `Reset` Long Hold (`>= 2.0 s`) |
| :--- | :--- | :--- | :--- | :--- |
| **1. Return Policy & Speed** | `Stup` | Cycles Return Policy:<br>`Auto` (Auto-Return) -> `HAnd` (Manual) -> `test` (Slow Measure) | Cycles Return Speed:<br>`-320` <-> `-240` <-> `-160` <-> `-80` RPM | *(ignored — resets idle timer)* |
| **2. Travel Distance** | `dist` | Increases `totalTravelDistanceMm_` by `+0.1 mm` (repeatable, max `187.5 mm`) | Decreases `totalTravelDistanceMm_` by `-0.1 mm` (repeatable, min `153.5 mm`) | *(ignored — auto-saved to EEPROM on 8s Setup exit or `FCAL` start)* |
| **3. Fault History** | `nonE` or last Fault (`StSt`, `StSP`, etc.) | *(no action)* | Clears stored Fault History in EEPROM (`nonE` + short beep) | *(ignored)* |
| **4. Adaptive Distance** | `Auto` -> shows `XXX.X mm` | *(no action)* | *(no action)* | Saves auto-tuned `totalTravelDistanceMm_` to EEPROM (**`SAVd`**) |
| **5. Fast Calibration** | `FCAL` | Launches Fast Calibration (requires Start switch active, else **`noHm`**) | Launches Fast Calibration (requires Start switch active, else **`noHm`**) | *(ignored)* |

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

### Method 2: Slow Tracking Calibration (`test` / `MEASURE_RT`, ~61 minutes)
Measures end-to-end travel under real tracking load at Sidereal speed:
1. In Setup Page 1 (`Stup`), press `Speed Up` twice until **`test`** appears, and wait `8 s` to exit to **`rdy`**.
2. Start from the Start switch by tapping **`Reset`** (let **`homE`** complete).
3. While tracking, the display shows **live traveled millimeters** instead of `MM:SS`.
4. When the Stop switch trips, the platform enters the same **`diSt`** <-> **`XXX.X`** review screen as Fast Calibration: hold **`Reset` `2.0 s`** to save (`SAVd`) or tap `Reset` to discard.

### Method 3: Closed-Loop Adaptive Auto-Calibration
Automatically fine-tunes the active travel distance immediately in RAM during normal observing nights:
- **Eligibility ("Clean Full Run"):** Only evaluates runs that:
  1. Started from a full home (`homE` or Start-switch arrival from `RETURN_ST`),
  2. Ran completely to the Stop switch without any user pause, fast-forward, rewind, or fault,
  3. Remained at **Sidereal** rate with zero manual speed adjustments (`0 µrad/min`).
- **Immediate RAM Adaptation & Filtering:** Ignores errors `< 0.1 mm` (deadband) and rejects outliers `> 5.0 mm` (e.g., if bumped). Applies a `33%` low-pass filter step directly to the live `totalTravelDistanceMm_` in RAM (clamped within `153.5–187.5 mm`) so the very next run immediately uses the improved travel distance.
- **End-of-Run Notifications (Before Pausing):**
  - Plays a **triple ascending chime** (`600` -> `800` -> `1000 Hz`) the moment the Stop switch triggers and an adjustment occurs.
  - During the **first 10 seconds of `Auto` return (`RETURN_ST`)**, shows `"rtrn"` for `2 s` and then alternates **`CAL `** <-> **`XXX.X`** (the new live distance in mm) every `1.0 s` before switching to the standard progress view (press `Speed Up` or `Speed Down` to skip straight to progress view).
  - In **`HAnd` mode (`WAIT_RETURN_ST`)**, alternates **`push`** <-> **`CAL `** every `1.5 s`.
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

### Clearing & Recovering from an Error (`ERROR_ST`)
1. **Mute the 30-second reminder chirp (optional):** Press **`Speed Up`** or **`Speed Down`**.
2. **Inspect the platform hardware:** Ensure switches are not stuck/shorted and mechanical travel is clear.
3. **Hold `Reset` for `2.0 seconds`:** Marks the fault record as acknowledged in EEPROM, clears the fault, resets position to unhomed (`rdy`), and emits a confirmation beep.
   - *(Note: If both switches are still physically shorted together, recovery is rejected with a `300 Hz` beep to protect EEPROM and mechanics until the short is cleared.)*
4. **Inspect or Clear Past Faults Later:** Open Setup (**Hold `Reset` 3.5 s**) and tap twice to Page 3 to view the last recorded fault mnemonic; press **`Speed Down`** to clear it to **`nonE`**.

