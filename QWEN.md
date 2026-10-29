# Project Overview: Line Follower UKIT

A PID-based line follower robot designed for the **uKit Explore 2** board. The project implements a modular Arduino sketch that handles line tracing, intersection navigation, and safety features.

## Architecture

The project follows a modular structure where the main `.ino` file acts as the state machine and orchestrator, while logic is encapsulated in specialized classes:

- **`LineFollowerFin.ino`**: Main entry point. Manages the high-level state machine (`setup` and `loop`).
- **`config.h`**: Centralized configuration for all tunable parameters (PID constants, speeds, thresholds, timeouts, and sensor IDs).
- **`status.h`**: Shared `Status` struct used for cross-module communication (e.g., `jalan`, `modeSetelKd`).
- **`sensor_garis`**: Handles IR sensor reading, exponential filtering, calibration, and EEPROM persistence of thresholds.
- **`penggerak`**: Implements PID computation, adaptive base speed, motor slew limiting, and movement helpers (e.g., 90° turns).
- **`antarmuka`**: Manages user interaction via onboard buttons, sound sensor (clap toggle), and Serial commands.
- **`pengaman`**: Implements safety checks including battery voltage monitoring, tilt detection (IMU), ultrasonic obstacle stop, and "stuck" detection.

## Building and Running

### Requirements
- **Hardware**: uKit Explore 2 board.
- **Software**: Arduino IDE.
- **Libraries**: `uKitExplore2` library (external) and the built-in `EEPROM.h`.

### Execution
1. Open `LineFollowerFin/LineFollowerFin.ino` in the Arduino IDE.
2. Ensure the `uKitExplore2` library is installed.
3. Compile and upload to the board.

### Verification
Since there are no automated test scripts, verification is performed by:
- Compiling the code in the Arduino IDE to ensure no syntax/linkage errors.
- Running the robot on a track and monitoring telemetry via the Serial Plotter.

## Development Conventions

### Coding Style & Language
- **Language**: Comments and Serial prints are written in **Indonesian**. Maintain this for consistency.
- **Structure**: Keep logic in `.cpp`/`.h` pairs. The `.ino` file should remain a lean coordinator.
- **Forward Declarations**: Because of the `eepromPutHemat` template in `sensor_garis.h`, explicit forward declarations in the `.ino` file (or `fwd.h`) are mandatory to prevent Arduino's auto-prototype generator from failing.

### Hardware Conventions
- **Sensors**: Left sensor is `readInfraredDistance(2)`, right is `readInfraredDistance(1)`.
- **Motors**: Left side (servos 4+2, dir `1`), Right side (servos 1+3, dir `0`).
- **Speed**: Derated to `0-150` for stability (though hardware allows `0-255`).
- **PID**: Error is calculated as `(left - right) / 2`.

### Onboard Tuning (No Laptop Required)
- **Button 1**:
  - Short press: Adjusts `Kp` (±1.0) or `Kd` (±0.5).
  - Long press: Toggles between `Kp`/`Kd` tuning mode (when stopped) or stops the robot (when running).
- **Button 2**:
  - Short press: Adjusts max speed (±10).
  - Long press: Saves current parameters to EEPROM and starts the robot.
- **Calibration**: Press **both buttons simultaneously** while stopped to trigger a 3-second auto-calibration of sensor thresholds.

### EEPROM Persistence
The following are stored in EEPROM (Magic value `123` at address 0):
- `Kp` (addr 4), `Kd` (addr 8), `kecepatanDasarMaks` (addr 16), `AMBANG_KIRI` (addr 20), `AMBANG_KANAN` (addr 22).
- *Note: `Ki` and `kecepatanDasarMin` are NOT persisted.*

## Key Constraints
- **Blocking Calls**: 90° turn helpers use `while` loops that block until the line is found. Do not refactor these into non-blocking code without ensuring the robot maintains its wait-state.
- **PID Interval**: PID calculations must occur at a constant interval (`PID_INTERVAL_MS = 10`) to ensure the Derivative (D) term remains stable.
