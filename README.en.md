# Automatic Shuttlecock Pickup and Stacking Robot (Software Part)

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
| Obstacle avoidance | A separate YOLO model detects blocking obstacles such as people and backpacks; stops and waits when blocked, gives up the target after a prolonged block |
| Battery monitoring | The firmware reports battery voltage every second; low-voltage warnings are throttled, critical voltage forces a stop, and the level shows on both screen and web |
| Web remote monitoring | Built-in MJPEG stream plus a status endpoint — open `http://<ip>:8000` in a browser to view the feed, action, battery, pickup count, and live session statistics remotely |
| Session statistics | Uptime, rolling FPS, pickup attempts/success/failure and success rate, escapes, and obstacle give-ups; shown live on the web page and printed as a summary on exit |
| Staged approach | In ground mode, within 1 m the robot automatically switches to a creep gear: lower speed with stronger turn gain, then pauses briefly to confirm alignment before picking up — prevents overshooting and improves alignment |
| Stuck detection & escape | If the target distance stops decreasing while driving, the robot is judged stuck and automatically backs up then turns in place to escape; too many escapes on the same ball triggers a cooldown and skips it |
| Safe shutdown | Ctrl+C or Q automatically stops the robot, turns off the pickup motor, and releases resources |
| Serial watchdog | Firmware-side 500ms command timeout auto-stop; heartbeat keeps the pickup motor alive during collection |
| Camera watchdog | No new frame for `CAMERA_STALE_TIMEOUT` seconds stops the robot immediately, so a frozen stream never drives on old frames; driving resumes after recovery |
| Startup preflight | Checks `config.py` value ranges, battery threshold ordering, model and calibration files before startup; errors block startup, warnings let it continue |

## Project Structure

```text
automatic_shuttlecock_pickup
│
├── main.py                  # Main program: main loop, action execution, debug overlay
├── config.py                # Centralized configuration (with per-feature switches)
├── preflight.py             # Startup checks: value ranges, battery ordering, model/calibration files
├── session_stats.py         # Run statistics: uptime, FPS, pickup success rate, escapes, give-ups
├── camera_watchdog.py       # Stale-stream detection (driver returning old frames)
├── camera.py                # Camera: background thread reads frames, keeps the latest only
├── detector.py              # YOLO detection + ByteTrack tracking + Kalman smoothing
├── kalman_filter.py         # Constant-velocity Kalman filter
├── decision.py              # Decision layer: PD control, pickup confirmation, multi-ball scheduling, obstacle avoidance
├── ground_projection.py     # Homography: image coordinates -> ground coordinates (meters)
├── robot_controller.py      # Serial command wrapper + background firmware report parsing (battery)
├── arduino_controller.ino   # Arduino firmware: PWM differential drive + watchdog + battery reporting
├── web_server.py            # Web remote monitoring: MJPEG stream + status JSON (optional)
├── camera_calibration.py    # Camera intrinsics calibration script (optional)
├── calibrate_homography.py  # Ground homography calibration script (optional)
├── tests/                   # Offline unit tests (preflight, session statistics)
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

Startup runs `preflight.py` first. Out-of-range values, `BATTERY_CRITICAL_MV` not below `BATTERY_LOW_MV`, a missing model, or a missing ground homography block startup. Missing drive calibration and obstacle-model reuse only print warnings. Camera, serial and log directories are not opened until the checks pass.

Run the offline unit tests (no camera, Arduino, or model inference required):

```bash
python -m unittest discover -s tests -v
```

## Configuration

All parameters live in `config.py`. Common ones:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `USE_SERIAL` | `False` | Whether to connect serial; keep False when tuning vision |
| `SERIAL_PORT` | `"COM3"` | Arduino serial port |
| `CONFIDENCE_THRESHOLD` | `0.6` | YOLO confidence threshold; raise it if false positives appear |
| `MAX_SPEED` | `200` | Motor PWM limit (0–255) |
| `CENTER_TOLERANCE` | `80` | Pixel-mode steering dead zone (pixels) |
| `CAMERA_STALE_TIMEOUT` | `2.0` | Frozen-camera timeout (seconds); no new frame for this long stops the robot |
| `USE_TRACKER` | `True` | ByteTrack tracking switch |
| `USE_KALMAN` | `True` | Kalman filter switch; False falls back to EMA |
| `USE_GROUND_PROJECTION` | `False` | Ground projection switch; calibrate first |
| `MULTI_BALL` | `True` | Multi-ball scheduling switch |
| `COLLECT_CONFIRM` | `True` | Visual pickup confirmation switch |
| `MAX_ACCEL` | `1200.0` | Wheel-speed acceleration limit (PWM/s), prevents slipping on sharp turns |
| `USE_OBSTACLE_AVOID` | `False` | Obstacle avoidance switch |
| `OBSTACLE_MODEL_PATH` | `None` | Obstacle model path; None reuses the shuttlecock model |
| `OBSTACLE_CLASSES` | `("person", ...)` | Obstacle classes (people, backpacks, etc.) |
| `OBSTACLE_STOP_DISTANCE_M` | `0.5` | Obstacle stop distance (ground mode, meters) |
| `OBSTACLE_AREA_NEAR` | `80000` | Obstacle proximity threshold (pixel mode, larger area = closer) |
| `OBSTACLE_BLOCK_TIMEOUT` | `8.0` | Prolonged-block timeout (seconds); the target is abandoned after it expires |
| `USE_BATTERY_MONITOR` | `False` | Battery monitoring switch |
| `BATTERY_LOW_MV` | `10500` | Low-voltage threshold (mV); a warning is raised below it |
| `BATTERY_CRITICAL_MV` | `10000` | Critical voltage threshold (mV); the robot force-stops below it |
| `USE_WEB_SERVER` | `False` | Web remote monitoring switch |
| `WEB_SERVER_PORT` | `8000` | Web monitoring port |
| `WEB_JPEG_QUALITY` | `70` | Web stream JPEG quality (1–100) |
| `USE_STAGED_APPROACH` | `True` | Staged approach switch (ground mode only) |
| `STAGED_NEAR_DISTANCE_M` | `1.0` | Near-range creep-gear distance threshold (meters) |
| `STAGED_NEAR_SPEED` | `60` | Max creep-gear forward speed (PWM) |
| `STAGED_KP_ANGLE` / `STAGED_KD_ANGLE` | `2.2 / 0.3` | Creep-gear bearing PD gains (sharper turn at low speed) |
| `STAGED_SETTLE_TIME` | `1.0` | Pause-to-observe duration (seconds) to confirm alignment before pickup |
| `USE_STUCK_DETECT` | `True` | Stuck detection switch (ground mode only) |
| `STUCK_DURATION` | `2.0` | Duration (seconds) of no progress before judging stuck |
| `STUCK_DISTANCE_EPS` | `0.15` | Distance-change tolerance (meters); less counts as no movement |
| `STUCK_REVERSE_SECONDS` / `STUCK_REVERSE_SPEED` | `0.5 / 80` | Escape reverse duration (s) and speed (PWM) |
| `STUCK_TURN_SECONDS` / `STUCK_TURN_SPEED` | `0.6 / 80` | Escape in-place turn duration (s) and speed (PWM) |
| `STUCK_MAX_ESCAPES` | `3` | Max escapes per target before cooldown |
| `STUCK_RETRY_COOLDOWN` | `8.0` | Cooldown after escape limit (seconds) |

## Workflow

1. **Detection**: YOLO detects shuttlecocks, ByteTrack assigns tracking IDs, Kalman smooths positions
2. **Chase**: PD control computes left/right wheel speeds; the robot aligns with the target and drives forward. In ground mode, within 1 m it automatically switches to a creep gear (lower speed, stronger turn gain) and pauses for 1 second to confirm alignment before pickup
3. **Pickup**: When the target is closer than the threshold (or 0.25 m in ground mode), the robot stops and runs the pickup mechanism for 2 seconds
4. **Confirmation**: While collecting, the system tracks whether the ball is still visible at its original spot to judge pickup success
5. **Next target**: On success, picks the next ball in the current frame and drives straight to it; on failure, retries in place, and after too many retries cools down for 10 seconds and skips that ball
6. **Search**: When nothing is in view, spins in place toward the direction where the ball was last seen
7. **Obstacle avoidance** (optional): While driving, if an obstacle such as a person blocks the path between the robot and the target, the robot stops and waits; after a block longer than 8 seconds it abandons the target and resumes searching
8. **Battery protection** (optional): Warns below 10.5V, force-stops the robot and turns off the pickup motor below 10V
9. **Stuck escape** (ground mode only): If the target distance fails to decrease for 2 seconds while driving, the robot backs up for 0.5 s and turns in place for 0.6 s, then drives toward the ball again; more than 3 escapes on the same ball triggers an 8-second cooldown and skips it

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

The firmware reports the battery voltage once per second: `BATT <millivolts>`, e.g. `BATT 11200` (11.2V).

## Notes

- `MIN_SPEED = 80` is currently unused; the effective low-speed compensation is the hardcoded value 40 in `decision.py` — tune against that on the real robot
- The legacy white-region detection is deprecated: white paper, walls, and clothing cause false positives. YOLO is the final approach
- Set `SQUARE_SIZE` to match your printed checkerboard size before ground calibration
- Battery monitoring requires a voltage divider (10k + 3.3k, ratio 4.03) between the battery and Arduino A0; update `VOLTAGE_DIVIDER_RATIO` in the firmware to match your actual resistors. The A0 input must never exceed 5V
- Obstacle avoidance reuses the shuttlecock model by default (which lacks person classes); train a separate model containing person/backpack classes and point `OBSTACLE_MODEL_PATH` at it for real use
- The web monitor is for LAN observation only, with no authentication — do not expose it to the public internet
