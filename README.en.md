# Automatic Shuttlecock Pickup and Stacking Robot

[English](README.en.md) | 中文

An automatic shuttlecock pickup robot built on YOLO vision and an Arduino differential-drive chassis. The robot finds shuttlecocks on the court, drives up to them, runs the pickup mechanism to collect them, and automatically moves on to the next one for continuous multi-ball operation.

## Features

| Module | Description |
|--------|-------------|
| Target detection | YOLOv8 shuttlecock detection with ByteTrack multi-object tracking and stable target ID locking |
| Trajectory smoothing | Constant-velocity Kalman filter on target position; extrapolates the trajectory during missed detections (up to 1 second) |
| Drive control | PD proportional differential steering, straight drive inside the dead zone, wheel-speed acceleration limiting against slipping, automatic low-speed compensation against motor stalling |
| Distance estimation | Two modes: pixel-area estimation (default) and homography-based ground projection (metric, requires calibration) |
| Multi-ball scheduling | After picking up one ball, switches directly to the next one in view without re-searching |
| Pickup confirmation | Visually confirms whether the ball was picked up; retries in place on failure, enters cooldown after too many retries to avoid getting stuck |
| Safe shutdown | Ctrl+C or Q automatically stops the robot, turns off the pickup motor, and releases resources |
| Serial watchdog | Firmware-side 500ms command timeout auto-stop; heartbeat keeps the pickup motor alive during collection |

## Project Structure

```text
automatic_shuttlecock_pickup
│
├── main.py                  # Main program: main loop, action execution, debug overlay
├── config.py                # Centralized configuration (with per-feature switches)
├── camera.py                # Camera: background thread reads frames, keeps the latest only
├── detector.py              # YOLO detection + ByteTrack tracking + Kalman smoothing
├── kalman_filter.py         # Constant-velocity Kalman filter
├── decision.py              # Decision layer: PD control, pickup confirmation, multi-ball scheduling
├── ground_projection.py     # Homography: image coordinates -> ground coordinates (meters)
├── robot_controller.py      # Serial command wrapper
├── arduino_controller.ino   # Arduino firmware: PWM differential drive + watchdog
├── camera_calibration.py    # Camera intrinsics calibration script (optional)
├── calibrate_homography.py  # Ground homography calibration script (optional)
├── requirements.txt
└── model/
    └── shuttlecock.pt       # YOLO shuttlecock model
```

## Installation

```bash
python -m pip install -r requirements.txt
```

Dependencies: Python 3.8+, ultralytics (YOLO), OpenCV, pyserial, numpy.

Place your trained YOLO model at `model/shuttlecock.pt`.

## Running

```bash
python main.py
```

- Press `Q` to quit, or Ctrl+C to interrupt (both stop the robot automatically)
- Serial is disabled by default (`USE_SERIAL = False`): the program only shows the video feed and control outputs, which is handy for tuning vision first
- For real runs, set `USE_SERIAL = True` and `SERIAL_PORT` in `config.py`

## Configuration

All parameters live in `config.py`. Common ones:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `USE_SERIAL` | `False` | Whether to connect serial; keep False when tuning vision |
| `SERIAL_PORT` | `"COM3"` | Arduino serial port |
| `CONFIDENCE_THRESHOLD` | `0.6` | YOLO confidence threshold; raise it if false positives appear |
| `MAX_SPEED` | `200` | Motor PWM limit (0–255) |
| `CENTER_TOLERANCE` | `80` | Pixel-mode steering dead zone (pixels) |
| `USE_TRACKER` | `True` | ByteTrack tracking switch |
| `USE_KALMAN` | `True` | Kalman filter switch; False falls back to EMA |
| `USE_GROUND_PROJECTION` | `False` | Ground projection switch; calibrate first |
| `MULTI_BALL` | `True` | Multi-ball scheduling switch |
| `COLLECT_CONFIRM` | `True` | Visual pickup confirmation switch |
| `MAX_ACCEL` | `1200.0` | Wheel-speed acceleration limit (PWM/s), prevents slipping on sharp turns |

## Workflow

1. **Detection**: YOLO detects shuttlecocks, ByteTrack assigns tracking IDs, Kalman smooths positions
2. **Chase**: PD control computes left/right wheel speeds; the robot aligns with the target and drives forward
3. **Pickup**: When the target is closer than the threshold (or 0.25 m in ground mode), the robot stops and runs the pickup mechanism for 2 seconds
4. **Confirmation**: While collecting, the system tracks whether the ball is still visible at its original spot to judge pickup success
5. **Next target**: On success, picks the next ball in the current frame and drives straight to it; on failure, retries in place, and after too many retries cools down for 10 seconds and skips that ball
6. **Search**: When nothing is in view, spins in place toward the direction where the ball was last seen

## Optional: Ground Projection Calibration

Ground projection closes the loop on real bearing angles and metric distances, decoupling the control from camera height and resolution — no re-tuning needed when the camera mount changes. To enable:

1. **Camera intrinsics**: Print a 9×6 checkerboard (matching the `SQUARE_SIZE` in `camera_calibration.py`), run `python camera_calibration.py`, capture at least 10 shots of the board from various angles (Space to shoot, Q to finish). This generates `calibration.npz`
2. **Homography**: Place the robot at the center of the court, run `python calibrate_homography.py`, click 4 known ground points in the frame (directly below the robot, straight ahead, front-right, right side), and enter the real distances. This generates `homography.npz`
3. Set `USE_GROUND_PROJECTION = True` in `config.py`

The robot works without calibration too: the default pixel mode estimates distance from box area — less accurate, but zero configuration.

## Serial Protocol

Baud rate 115200, commands terminated by a newline.

| Command | Description |
|---------|-------------|
| `SPEED <L> <R>` | Left/right wheel PWM speed, 0–255 |
| `STOP` | Stop |
| `COLLECT_ON` / `COLLECT_OFF` | Pickup motor on/off |
| `TURN_LEFT` / `TURN_RIGHT` | Spin in place (for searching) |
| `PING` | Heartbeat, refreshes the watchdog only |

The firmware watchdog auto-stops the robot and turns off the pickup motor if no command arrives within 500 ms; it does not trigger before the first command after power-up.

## Notes

- `MIN_SPEED = 80` is currently unused; the effective low-speed compensation is the hardcoded value 40 in `decision.py` — tune against that on the real robot
- The legacy white-region detection is deprecated: white paper, walls, and clothing cause false positives. YOLO is the final approach
- Set `SQUARE_SIZE` to match your printed checkerboard size before ground calibration
