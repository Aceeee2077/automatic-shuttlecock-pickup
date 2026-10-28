"""Startup configuration checks.

The robot reads every threshold from config.py. A typo there (for example a
critical battery voltage above the low-voltage warning, or a missing model
file) is cheaper to catch before the camera and serial port are opened than
after the robot has started driving.

`validate_config()` is deliberately independent of OpenCV, YOLO and pyserial
so it can run in unit tests and in a headless checkout.
"""

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional


@dataclass(frozen=True)
class CheckIssue:
    level: str          # "error" or "warning"
    setting: str
    message: str

    def line(self):
        return "[%s] %s: %s" % (self.level.upper(), self.setting, self.message)


@dataclass
class PreflightReport:
    errors: List[CheckIssue] = field(default_factory=list)
    warnings: List[CheckIssue] = field(default_factory=list)

    @property
    def ok(self):
        return not self.errors

    def error(self, setting, message):
        self.errors.append(CheckIssue("error", setting, message))

    def warn(self, setting, message):
        self.warnings.append(CheckIssue("warning", setting, message))

    def lines(self):
        return [issue.line() for issue in self.errors + self.warnings]


def _number(report, cfg, name, low, high):
    """Read a numeric setting and add an error when it is outside [low, high]."""
    raw = getattr(cfg, name, None)

    try:
        value = float(raw)
    except (TypeError, ValueError):
        report.error(name, "must be a number, got %r" % (raw,))
        return None

    if not math.isfinite(value):
        report.error(name, "must be finite, got %r" % (raw,))
        return None

    if value < low or value > high:
        report.error(name, "must be between %s and %s, got %s" % (low, high, raw))
        return None

    return value


def _exists(path, base_dir, file_exists):
    candidate = Path(path)

    if not candidate.is_absolute():
        candidate = Path(base_dir) / candidate

    return file_exists(candidate)


def validate_config(cfg, base_dir=".", file_exists=None):
    """Validate config.py values and calibration/model paths.

    Returns a PreflightReport. Errors block startup; warnings are printed but
    let the run continue. `file_exists` is injectable for tests.
    """
    report = PreflightReport()
    file_exists = file_exists or (lambda path: Path(path).is_file())

    # ---- camera and serial ----
    camera_index = getattr(cfg, "CAMERA_INDEX", 0)
    if not isinstance(camera_index, int) or camera_index < 0:
        report.error("CAMERA_INDEX", "must be a non-negative integer")

    _number(report, cfg, "FRAME_WIDTH", 1, 10000)
    _number(report, cfg, "FRAME_HEIGHT", 1, 10000)
    _number(report, cfg, "CAMERA_STALE_TIMEOUT", 0.1, 60)
    _number(report, cfg, "BAUD_RATE", 1, 4000000)

    use_serial = bool(getattr(cfg, "USE_SERIAL", False))
    port = str(getattr(cfg, "SERIAL_PORT", "") or "").strip()

    if use_serial and not port:
        report.error("SERIAL_PORT", "cannot be empty when USE_SERIAL is True")

    # ---- steering and detection ----
    _number(report, cfg, "MAX_SPEED", 1, 255)
    _number(report, cfg, "FORWARD_SPEED", 1, 255)
    _number(report, cfg, "CENTER_TOLERANCE", 0, 10000)
    _number(report, cfg, "CONFIDENCE_THRESHOLD", 0, 1)
    _number(report, cfg, "COLLECT_TIME", 0.1, 60)
    _number(report, cfg, "COLLECT_DISTANCE", 0.01, 1000)
    _number(report, cfg, "MISS_FRAME_LIMIT", 1, 1000)

    if bool(getattr(cfg, "USE_ACCEL_LIMIT", True)):
        _number(report, cfg, "MAX_ACCEL", 1, 100000)

    # ---- collection confirmation ----
    _number(report, cfg, "COLLECT_CONFIRM_RATIO", 0, 1)
    _number(report, cfg, "COLLECT_CONFIRM_RADIUS", 0, 10000)
    _number(report, cfg, "MAX_COLLECT_RETRIES", 0, 100)
    _number(report, cfg, "COLLECT_RETRY_COOLDOWN", 0, 3600)

    # ---- ground projection and staged approach ----
    use_ground = bool(getattr(cfg, "USE_GROUND_PROJECTION", False))

    if use_ground:
        homography = getattr(cfg, "HOMOGRAPHY_PATH", "")
        if not homography:
            report.error("HOMOGRAPHY_PATH", "cannot be empty in ground mode")
        elif not _exists(homography, base_dir, file_exists):
            report.error(
                "HOMOGRAPHY_PATH",
                "%s not found; run calibrate_homography.py or disable ground mode"
                % homography
            )

        calibration = getattr(cfg, "CALIBRATION_PATH", "")
        if calibration and not _exists(calibration, base_dir, file_exists):
            report.warn(
                "CALIBRATION_PATH",
                "%s not found; frames will be used without undistortion"
                % calibration
            )

        _number(report, cfg, "ANGLE_TOLERANCE", 0.001, math.pi)
        _number(report, cfg, "KP_ANGLE", 0, 100)
        _number(report, cfg, "KD_ANGLE", 0, 100)
        _number(report, cfg, "COLLECT_DISTANCE_M", 0.01, 100)

        if bool(getattr(cfg, "USE_STAGED_APPROACH", False)):
            _number(report, cfg, "STAGED_NEAR_DISTANCE_M", 0.01, 100)
            _number(report, cfg, "STAGED_NEAR_SPEED", 1, 255)
            _number(report, cfg, "STAGED_KP_ANGLE", 0, 100)
            _number(report, cfg, "STAGED_KD_ANGLE", 0, 100)
            _number(report, cfg, "STAGED_SETTLE_TIME", 0, 60)

        if bool(getattr(cfg, "USE_STUCK_DETECT", False)):
            _number(report, cfg, "STUCK_DURATION", 0.1, 60)
            _number(report, cfg, "STUCK_DISTANCE_EPS", 0, 100)
            _number(report, cfg, "STUCK_REVERSE_SECONDS", 0, 10)
            _number(report, cfg, "STUCK_REVERSE_SPEED", 1, 255)
            _number(report, cfg, "STUCK_TURN_SECONDS", 0, 10)
            _number(report, cfg, "STUCK_TURN_SPEED", 1, 255)
            _number(report, cfg, "STUCK_MAX_ESCAPES", 0, 100)
            _number(report, cfg, "STUCK_RETRY_COOLDOWN", 0, 3600)

    # ---- obstacle avoidance ----
    use_obstacle = bool(getattr(cfg, "USE_OBSTACLE_AVOID", False))

    if use_obstacle:
        obstacle_path = getattr(cfg, "OBSTACLE_MODEL_PATH", None)
        model_path = obstacle_path or getattr(cfg, "YOLO_MODEL_PATH", "")

        if not model_path:
            report.error(
                "OBSTACLE_MODEL_PATH",
                "no model configured for obstacle detection"
            )
        elif not _exists(model_path, base_dir, file_exists):
            report.error(
                "OBSTACLE_MODEL_PATH",
                "%s not found" % model_path
            )

        if obstacle_path is None:
            report.warn(
                "OBSTACLE_MODEL_PATH",
                "reusing the shuttlecock model; it must contain the classes in "
                "OBSTACLE_CLASSES"
            )

        _number(report, cfg, "OBSTACLE_CONF_THRESHOLD", 0, 1)
        _number(report, cfg, "OBSTACLE_STOP_DISTANCE_M", 0.01, 100)
        _number(report, cfg, "OBSTACLE_AREA_NEAR", 0, 100000000)
        _number(report, cfg, "OBSTACLE_BLOCK_TIMEOUT", 0.1, 3600)

    # ---- battery ----
    if bool(getattr(cfg, "USE_BATTERY_MONITOR", False)):
        low = _number(report, cfg, "BATTERY_LOW_MV", 1000, 60000)
        critical = _number(report, cfg, "BATTERY_CRITICAL_MV", 1000, 60000)

        if low is not None and critical is not None and critical >= low:
            report.error(
                "BATTERY_CRITICAL_MV",
                "must be lower than BATTERY_LOW_MV (%s)" % low
            )

        if not use_serial:
            report.warn(
                "USE_BATTERY_MONITOR",
                "battery reporting needs USE_SERIAL = True"
            )

    # ---- web monitor ----
    if bool(getattr(cfg, "USE_WEB_SERVER", False)):
        _number(report, cfg, "WEB_SERVER_PORT", 1, 65535)
        _number(report, cfg, "WEB_JPEG_QUALITY", 1, 100)

    # ---- run logger ----
    if bool(getattr(cfg, "USE_RUN_LOGGER", False)):
        _number(report, cfg, "RUN_LOG_JPEG_QUALITY", 1, 100)
        every_n = _number(report, cfg, "RUN_LOG_EVERY_N_FRAMES", 1, 1000)

        if every_n is not None and int(every_n) != every_n:
            report.error(
                "RUN_LOG_EVERY_N_FRAMES",
                "must be a whole number, got %s" % every_n
            )

        if not str(getattr(cfg, "RUN_LOG_DIR", "") or "").strip():
            report.error("RUN_LOG_DIR", "cannot be empty")

    # ---- model and optional drive calibration ----
    model_path = getattr(cfg, "YOLO_MODEL_PATH", "")

    if not model_path:
        report.error("YOLO_MODEL_PATH", "cannot be empty")
    elif not _exists(model_path, base_dir, file_exists):
        report.error("YOLO_MODEL_PATH", "%s not found" % model_path)

    if bool(getattr(cfg, "USE_DRIVE_CALIBRATION", True)):
        calibration_path = getattr(cfg, "DRIVE_CALIBRATION_PATH", "")

        if not calibration_path:
            report.warn(
                "DRIVE_CALIBRATION_PATH",
                "empty path; wheels will use the same speed"
            )
        elif not _exists(calibration_path, base_dir, file_exists):
            report.warn(
                "DRIVE_CALIBRATION_PATH",
                "%s not found; wheels will use the same speed" % calibration_path
            )

    return report


def print_report(report, stream=None):
    """Print a compact preflight report. Returns True when startup is allowed."""
    import sys

    stream = stream or sys.stdout

    if report.errors:
        print(
            "Preflight failed with %d error(s). Fix config.py before starting."
            % len(report.errors),
            file=stream
        )
    elif report.warnings:
        print("Preflight passed with %d warning(s)." % len(report.warnings),
              file=stream)
    else:
        print("Preflight passed.", file=stream)

    for issue in report.errors + report.warnings:
        print("  " + issue.line(), file=stream)

    return report.ok
