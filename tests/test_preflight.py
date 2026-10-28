import io
import unittest
from pathlib import Path
from types import SimpleNamespace

from preflight import print_report, validate_config


def base_config(**overrides):
    values = dict(
        CAMERA_INDEX=0,
        USE_SERIAL=False,
        SERIAL_PORT="COM3",
        BAUD_RATE=115200,
        FRAME_WIDTH=1280,
        FRAME_HEIGHT=720,
        CAMERA_STALE_TIMEOUT=2.0,
        MAX_SPEED=200,
        FORWARD_SPEED=200,
        CENTER_TOLERANCE=80,
        CONFIDENCE_THRESHOLD=0.6,
        COLLECT_TIME=2.0,
        COLLECT_DISTANCE=2.0,
        MISS_FRAME_LIMIT=10,
        USE_ACCEL_LIMIT=True,
        MAX_ACCEL=1200.0,
        COLLECT_CONFIRM_RATIO=0.5,
        COLLECT_CONFIRM_RADIUS=150,
        MAX_COLLECT_RETRIES=2,
        COLLECT_RETRY_COOLDOWN=10.0,
        USE_GROUND_PROJECTION=False,
        HOMOGRAPHY_PATH="homography.npz",
        CALIBRATION_PATH="calibration.npz",
        ANGLE_TOLERANCE=0.12,
        KP_ANGLE=1.6,
        KD_ANGLE=0.3,
        COLLECT_DISTANCE_M=0.25,
        USE_STAGED_APPROACH=True,
        STAGED_NEAR_DISTANCE_M=1.0,
        STAGED_NEAR_SPEED=60,
        STAGED_KP_ANGLE=2.2,
        STAGED_KD_ANGLE=0.3,
        STAGED_SETTLE_TIME=1.0,
        USE_STUCK_DETECT=True,
        STUCK_DURATION=2.0,
        STUCK_DISTANCE_EPS=0.15,
        STUCK_REVERSE_SECONDS=0.5,
        STUCK_REVERSE_SPEED=80,
        STUCK_TURN_SECONDS=0.6,
        STUCK_TURN_SPEED=80,
        STUCK_MAX_ESCAPES=3,
        STUCK_RETRY_COOLDOWN=8.0,
        USE_OBSTACLE_AVOID=False,
        OBSTACLE_MODEL_PATH=None,
        OBSTACLE_CONF_THRESHOLD=0.5,
        OBSTACLE_STOP_DISTANCE_M=0.5,
        OBSTACLE_AREA_NEAR=80000,
        OBSTACLE_BLOCK_TIMEOUT=8.0,
        USE_BATTERY_MONITOR=False,
        USE_WEB_SERVER=False,
        USE_RUN_LOGGER=True,
        RUN_LOG_JPEG_QUALITY=85,
        RUN_LOG_EVERY_N_FRAMES=1,
        RUN_LOG_DIR="runs/replay",
        YOLO_MODEL_PATH="model/shuttlecock.pt",
        USE_DRIVE_CALIBRATION=True,
        DRIVE_CALIBRATION_PATH="drive_calibration.json",
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def validate(cfg, existing=("model/shuttlecock.pt", "drive_calibration.json"),
             base_dir="."):
    existing = {str(Path(p)) for p in existing}
    return validate_config(
        cfg,
        base_dir=base_dir,
        file_exists=lambda path: str(path) in existing,
    )


def settings_with_issue(report, level):
    issues = report.errors if level == "error" else report.warnings
    return {issue.setting for issue in issues}


class PreflightTests(unittest.TestCase):
    def test_valid_config_passes(self):
        report = validate(base_config())

        self.assertTrue(report.ok)
        self.assertEqual(report.errors, [])
        self.assertEqual(report.warnings, [])

    def test_camera_stale_timeout_must_be_positive(self):
        report = validate(base_config(CAMERA_STALE_TIMEOUT=0))

        self.assertFalse(report.ok)
        self.assertIn(
            "CAMERA_STALE_TIMEOUT", settings_with_issue(report, "error")
        )

    def test_non_numeric_threshold_is_an_error(self):
        report = validate(base_config(CONFIDENCE_THRESHOLD="high"))

        self.assertFalse(report.ok)
        self.assertIn("CONFIDENCE_THRESHOLD", settings_with_issue(report, "error"))

    def test_battery_critical_must_be_lower_than_low(self):
        report = validate(base_config(
            USE_BATTERY_MONITOR=True,
            BATTERY_LOW_MV=10000,
            BATTERY_CRITICAL_MV=10500,
        ))

        self.assertFalse(report.ok)
        self.assertIn("BATTERY_CRITICAL_MV", settings_with_issue(report, "error"))

    def test_missing_model_blocks_startup(self):
        report = validate(base_config(), existing=())

        self.assertFalse(report.ok)
        self.assertIn("YOLO_MODEL_PATH", settings_with_issue(report, "error"))

    def test_ground_mode_needs_homography(self):
        report = validate(base_config(USE_GROUND_PROJECTION=True))

        self.assertFalse(report.ok)
        self.assertIn("HOMOGRAPHY_PATH", settings_with_issue(report, "error"))

    def test_ground_mode_accepts_existing_homography(self):
        report = validate(
            base_config(USE_GROUND_PROJECTION=True),
            existing=(
                "model/shuttlecock.pt",
                "drive_calibration.json",
                "homography.npz",
            ),
        )

        self.assertTrue(report.ok)

    def test_obstacle_model_reuse_only_warns(self):
        report = validate(base_config(USE_OBSTACLE_AVOID=True))

        self.assertTrue(report.ok)
        self.assertIn(
            "OBSTACLE_MODEL_PATH", settings_with_issue(report, "warning")
        )

    def test_battery_monitor_without_serial_warns(self):
        report = validate(base_config(
            USE_BATTERY_MONITOR=True,
            BATTERY_LOW_MV=10500,
            BATTERY_CRITICAL_MV=10000,
        ))

        self.assertTrue(report.ok)
        self.assertIn("USE_BATTERY_MONITOR", settings_with_issue(report, "warning"))

    def test_missing_drive_calibration_warns(self):
        report = validate(
            base_config(), existing=("model/shuttlecock.pt",)
        )

        self.assertTrue(report.ok)
        self.assertIn(
            "DRIVE_CALIBRATION_PATH", settings_with_issue(report, "warning")
        )

    def test_web_port_range_is_checked(self):
        report = validate(base_config(
            USE_WEB_SERVER=True,
            WEB_SERVER_PORT=70000,
            WEB_JPEG_QUALITY=70,
        ))

        self.assertFalse(report.ok)
        self.assertIn("WEB_SERVER_PORT", settings_with_issue(report, "error"))

    def test_frame_interval_must_be_whole(self):
        report = validate(base_config(RUN_LOG_EVERY_N_FRAMES=1.5))

        self.assertFalse(report.ok)
        self.assertIn(
            "RUN_LOG_EVERY_N_FRAMES", settings_with_issue(report, "error")
        )

    def test_print_report_returns_startup_decision(self):
        out = io.StringIO()
        self.assertTrue(print_report(validate(base_config()), stream=out))
        self.assertIn("Preflight passed", out.getvalue())

        out = io.StringIO()
        self.assertFalse(print_report(validate(base_config(), existing=()), stream=out))
        self.assertIn("Preflight failed", out.getvalue())


if __name__ == "__main__":
    unittest.main()
