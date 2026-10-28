"""Keep the run-log directory from growing without bound.

Every run creates runs/replay/run_<timestamp>/ with one JPEG per frame, so
an afternoon of tuning can easily produce several gigabytes. The retention
policy keeps the newest N runs and removes older ones at startup.

Only directories that follow the logger's own layout (`run_*` containing
log.jsonl) are candidates; unrelated files under the root are never touched.
"""

import shutil
from pathlib import Path

RUN_PREFIX = "run_"
RUN_MARKER = "log.jsonl"


def list_runs(root_dir):
    """Return run directories under root_dir, oldest first.

    Names sort chronologically (run_YYYYmmdd_HHMMSS), so the name is used
    instead of mtime; copying or restoring a directory does not reorder it.
    """
    root = Path(root_dir)

    if not root.is_dir():
        return []

    return sorted(
        (
            path for path in root.iterdir()
            if path.is_dir()
            and path.name.startswith(RUN_PREFIX)
            and (path / RUN_MARKER).is_file()
        ),
        key=lambda path: path.name,
    )


def prune_runs(root_dir, keep, remove=None):
    """Remove the oldest runs so at most `keep` remain.

    `keep <= 0` disables pruning (keep everything). `remove` is injectable
    for tests; it receives one path. Returns the removed paths. A failed
    removal is skipped so one locked directory cannot stop startup.
    """
    keep = int(keep)

    if keep <= 0:
        return []

    remove = remove or shutil.rmtree
    runs = list_runs(root_dir)
    stale = runs[:-keep] if len(runs) > keep else []
    removed = []

    for path in stale:
        try:
            remove(path)
        except OSError as e:
            print(f"Could not remove old run log {path}: {e}")
            continue

        removed.append(path)

    return removed
