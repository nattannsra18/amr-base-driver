"""Cross-process ownership for Nav2 and SLAM lifecycle transitions."""

from __future__ import annotations

import fcntl
import os
from pathlib import Path
import time


DEFAULT_LIFECYCLE_LOCK = (
    '/run/lock/indoor-delivery-robot-navigation-lifecycle.lock'
)


class LifecycleInterlock:
    """Hold one advisory lease across a complete lifecycle transition."""

    def __init__(self, path: str = DEFAULT_LIFECYCLE_LOCK) -> None:
        self.path = Path(path)
        self._descriptor: int | None = None

    @property
    def held(self) -> bool:
        return self._descriptor is not None

    def try_acquire(self) -> bool:
        if self.held:
            return True
        self.path.parent.mkdir(parents=True, exist_ok=True)
        try:
            try:
                descriptor = os.open(
                    self.path,
                    os.O_CREAT | os.O_EXCL | os.O_RDWR
                    | os.O_CLOEXEC | os.O_NOFOLLOW,
                    0o666,
                )
                created = True
            except FileExistsError:
                descriptor = os.open(
                    self.path,
                    os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW,
                )
                created = False
        except PermissionError:
            # The other service may be between creating the inode under its
            # restrictive umask and widening it below. Treat that tiny window
            # exactly like a busy lock so the caller retries safely.
            return False
        # Navigation runs as ``odroid`` while the Robot Agent runs as the
        # restricted ``indoor-robot`` account.  Both must open the same
        # advisory lock.  chmod the empty lock inode after creation because
        # the Agent's systemd UMask=0077 would otherwise make it private.
        if created:
            os.fchmod(descriptor, 0o666)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(descriptor)
            return False
        self._descriptor = descriptor
        return True

    def acquire(
        self,
        *,
        timeout_seconds: float,
        poll_seconds: float = 0.05,
    ) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_seconds)
        while True:
            if self.try_acquire():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(max(0.01, poll_seconds))

    def release(self) -> None:
        descriptor = self._descriptor
        self._descriptor = None
        if descriptor is None:
            return
        try:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)

    def __enter__(self) -> LifecycleInterlock:
        if not self.try_acquire():
            raise RuntimeError('Navigation lifecycle transition is busy')
        return self

    def __exit__(self, *_args: object) -> None:
        self.release()
