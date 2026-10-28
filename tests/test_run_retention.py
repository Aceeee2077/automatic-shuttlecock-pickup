import os
import shutil
import tempfile
import unittest
from pathlib import Path

from run_retention import list_runs, prune_runs


class RunRetentionTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def make_run(self, name, with_log=True):
        path = self.root / name
        path.mkdir()

        if with_log:
            (path / "log.jsonl").write_text("{}\n", encoding="utf-8")

        return path

    def names(self):
        return [path.name for path in list_runs(self.root)]

    def test_lists_only_real_run_directories(self):
        self.make_run("run_20260101_120000")
        self.make_run("run_20260102_120000", with_log=False)
        self.make_run("not_a_run")
        (self.root / "run_20260103_120000.txt").write_text("x", encoding="utf-8")

        self.assertEqual(self.names(), ["run_20260101_120000"])

    def test_keeps_newest_runs(self):
        for day in range(1, 6):
            self.make_run("run_202601%02d_120000" % day)

        removed = prune_runs(self.root, keep=2)

        self.assertEqual(
            [path.name for path in removed],
            [
                "run_20260101_120000",
                "run_20260102_120000",
                "run_20260103_120000",
            ],
        )
        self.assertEqual(
            self.names(),
            ["run_20260104_120000", "run_20260105_120000"],
        )

    def test_zero_keeps_everything(self):
        for day in range(1, 4):
            self.make_run("run_202601%02d_120000" % day)

        self.assertEqual(prune_runs(self.root, keep=0), [])
        self.assertEqual(len(self.names()), 3)

    def test_fewer_runs_than_keep_is_a_no_op(self):
        self.make_run("run_20260101_120000")

        self.assertEqual(prune_runs(self.root, keep=10), [])
        self.assertTrue((self.root / "run_20260101_120000").is_dir())

    def test_order_uses_names_not_mtime(self):
        old = self.make_run("run_20260101_120000")
        new = self.make_run("run_20260102_120000")

        # Make the older run look newer on disk; the name must still win.
        os.utime(old, (2_000_000_000, 2_000_000_000))
        os.utime(new, (1_000_000_000, 1_000_000_000))

        prune_runs(self.root, keep=1)

        self.assertFalse(old.exists())
        self.assertTrue(new.exists())

    def test_missing_root_is_safe(self):
        missing = self.root / "does_not_exist"

        self.assertEqual(list_runs(missing), [])
        self.assertEqual(prune_runs(missing, keep=1), [])

    def test_failed_removal_does_not_stop_the_rest(self):
        for day in range(1, 4):
            self.make_run("run_202601%02d_120000" % day)

        def remove(path):
            if path.name == "run_20260101_120000":
                raise OSError("in use")
            shutil.rmtree(path)

        removed = prune_runs(self.root, keep=1, remove=remove)

        self.assertEqual(
            [path.name for path in removed], ["run_20260102_120000"]
        )
        self.assertTrue((self.root / "run_20260101_120000").exists())
        self.assertTrue((self.root / "run_20260103_120000").exists())


if __name__ == "__main__":
    unittest.main()
