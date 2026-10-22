import stat

from amr_base_driver import lifecycle_interlock as interlock_module
from amr_base_driver.lifecycle_interlock import LifecycleInterlock


def test_interlock_excludes_another_owner_and_releases(tmp_path):
    path = tmp_path / 'navigation-lifecycle.lock'
    first = LifecycleInterlock(str(path))
    second = LifecycleInterlock(str(path))

    assert first.try_acquire() is True
    assert first.held is True
    assert second.try_acquire() is False

    first.release()

    assert first.held is False
    assert second.try_acquire() is True
    second.release()


def test_interlock_timeout_is_bounded(tmp_path):
    path = tmp_path / 'navigation-lifecycle.lock'
    first = LifecycleInterlock(str(path))
    second = LifecycleInterlock(str(path))
    assert first.try_acquire() is True

    assert second.acquire(timeout_seconds=0.0) is False

    first.release()


def test_interlock_is_openable_by_navigation_and_agent_accounts(tmp_path):
    path = tmp_path / 'navigation-lifecycle.lock'
    interlock = LifecycleInterlock(str(path))

    assert interlock.try_acquire() is True
    assert stat.S_IMODE(path.stat().st_mode) == 0o666

    interlock.release()


def test_interlock_treats_cross_account_creation_window_as_busy(
        tmp_path, monkeypatch):
    path = tmp_path / 'navigation-lifecycle.lock'
    interlock = LifecycleInterlock(str(path))

    monkeypatch.setattr(
        interlock_module.os,
        'open',
        lambda *_args, **_kwargs: (_ for _ in ()).throw(PermissionError()),
    )

    assert interlock.try_acquire() is False
    assert interlock.held is False


def test_existing_shared_lock_does_not_require_chmod(tmp_path, monkeypatch):
    path = tmp_path / 'navigation-lifecycle.lock'
    path.touch(mode=0o666)
    path.chmod(0o666)
    interlock = LifecycleInterlock(str(path))
    monkeypatch.setattr(
        interlock_module.os,
        'fchmod',
        lambda *_args: (_ for _ in ()).throw(
            AssertionError('only the lock creator may chmod the inode')
        ),
    )

    assert interlock.try_acquire() is True

    interlock.release()
