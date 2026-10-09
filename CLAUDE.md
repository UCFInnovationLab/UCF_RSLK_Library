# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An Energia/Arduino library (`UCF_RSLK`) providing drivers and a simplified API for the TI Robotic System Learning Kit (RSLK Max), built on the MSP-EXP432P401R LaunchPad. There is no application entry point — this repo is a hardware library consumed by `.ino` sketches (see `examples/`), either standalone in Energia/Arduino IDE or as a curriculum library for a robotics course.

## Build / test / lint

There is no command-line build, test, or lint tooling in this repo (no Makefile, package.json, CMake, or CI config). Compilation happens exclusively through the Energia/Arduino IDE:

1. Install Energia IDE (or Arduino IDE 1.8.13) with the Energia MSP432 board package (`package_energia_index.json`), board = "Red LaunchPad MSP432P401R EMT".
2. Add this repo as a library (Sketch > Include Library > Add .ZIP Library, or place it under the Energia/Arduino `libraries/` folder).
3. Install the "BNO055 by Robert Bosch GMBH" library dependency via Library Manager (required for gyro examples like `AirBag`, `GyroDrive`, `Robot_Challenge`).
4. Open any `examples/*/*.ino` sketch and Verify/Upload from the IDE.

There is no way to compile or test this code from the command line — to "verify a change works," reason about it by reading the code (pin mappings, state machines, calibration math) rather than claiming it was tested. If asked to validate, say explicitly that it requires flashing physical hardware via the IDE.

Docs are generated via Doxygen (`Doxyfile`, `INPUT = src/`, `OUTPUT_DIRECTORY = docs/`) and published to GitHub Pages from `docs/`. Regenerate with `doxygen Doxyfile` after changing headers in `src/`, and commit the resulting `docs/` output if asked to update docs.

## Architecture

### Layering

- **`src/SimpleRSLK.h`/`.cpp`** — the primary, recommended API (per README). Free functions (`setupRSLK()`, `readLineSensor()`, `setMotorSpeed()`, `getLinePosition()`, `isBumpSwitchPressed()`, `readSharpDist()`, button/LED helpers) that wrap the lower-level peripheral classes below. Most example sketches only need this header.
- **Peripheral-specific classes**, each independently usable:
  - `Bump_Switch` — single bump switch
  - `Romi_Motor_Power` — single motor (direction/speed/sleep)
  - `Encoder` — the two onboard wheel encoders (interrupt-driven tick counting)
  - `GP2Y0A21_Sensor` — Sharp IR distance sensor
  - `QTRSensors` / `QTRSensorsUCF` — Pololu QTR-8RC line sensor array; `QTRSensorsUCF` is a UCF-specific variant/extension of the upstream `QTRSensors` library code
- **`src/RSLK_Pins.h`** — the single source of truth for all Energia pin-number-to-LaunchPad-pin mappings (motors, encoders, bump switches, line sensor, servos, LCD, onboard LEDs/buttons). It is compiled conditionally:
  - `#if defined(USING_RSLK_CLASSIC)` selects pin mappings for the older RSLK Classic chassis.
  - `#else` (default) selects mappings for RSLK Max.
  When touching pin definitions, preserve both branches and keep the Energia-pin-number / Launchpad-pin comment (`// <- Energia Pin # Launchpad Pin -> Pxx.y`) — it's the only documentation of the physical wiring.
- **IMU/motion vendor drivers** — `Af_BNO055`, `Af_I2CDevice`, `Af_Sensor` (Adafruit-derived BNO055 orientation sensor support) and `DFRobot_BMI160` (accelerometer/gyro) are largely vendored third-party drivers under this library's namespace, plus `src/utility/` (`imumaths.h`, `matrix.h`, `quaternion.h`, `vector.h`) providing the math types they depend on. Treat these as integration points rather than code to redesign — match the vendor's existing conventions when patching them.

### Conventions specific to this codebase

- Public APIs use plain C-style constants instead of enums for mode/direction selectors, e.g. `LEFT_MOTOR`/`RIGHT_MOTOR`/`BOTH_MOTORS`, `MOTOR_DIR_FORWARD`/`MOTOR_DIR_BACKWARD`, `DARK_LINE`/`LIGHT_LINE` (defined in `SimpleRSLK.h`). Follow this pattern rather than introducing enums when extending the simplified API.
- Doxygen-style `///` / `/** */` comments with `\param[in]`/`\param[out]`/`\return` are used throughout the public headers (`SimpleRSLK.h`, `RSLK_Pins.h`) — match this style for new public API documentation since it feeds the published Doxygen site.
- `keywords.txt` drives Arduino IDE syntax highlighting (KEYWORD1 = types/classes, KEYWORD2 = methods/functions, LITERAL1 = constants). When adding new public classes/functions/constants intended for sketch authors, add corresponding entries.
- `library.properties` declares `architectures=msp432r` — this library is not intended to be architecture-portable; don't add guards for other MCU families.

### Examples (`examples/`)

Each subdirectory is a standalone sketch demonstrating one capability or a course exercise (bump switches, line following, encoders, IMU/gyro, servos). Several encode a simple state-machine pattern (`enum State { ... }` + `switch(state)` in `loop()`) for multi-stage autonomous behavior — see `Robot_Challenge/Robot_Challenge.ino` and `BumpSwitchMaze` for the canonical shape (state enum, per-state handler, sensor-driven transitions, PID-style line-following via `getLinePosition()`/proportional speed correction). `CalibrateLineSensor` is a prerequisite step for the min/max sensor calibration arrays (`sensorMinVal`/`sensorMaxVal`) hardcoded into the follow-line examples — those numeric arrays are per-robot/per-lighting-condition and are expected to be replaced by users, not treated as bugs.
