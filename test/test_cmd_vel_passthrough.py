from amr_base_driver.cmd_vel_passthrough import CommandArbiter, CommandOwner


def test_recovery_preempts_navigation_then_expires():
    arbiter = CommandArbiter(lease_seconds=0.2)
    accepted, changed = arbiter.accept(CommandOwner.NAVIGATION, 1.0)
    assert accepted and changed
    accepted, changed = arbiter.accept(CommandOwner.RECOVERY, 1.0)
    assert accepted and changed
    accepted, changed = arbiter.accept(CommandOwner.NAVIGATION, 1.1)
    assert not accepted and not changed
    assert not arbiter.expire(1.19)
    assert arbiter.expire(1.20)
    assert arbiter.owner == CommandOwner.NAVIGATION


def test_recovery_stream_renews_lease_then_stops():
    arbiter = CommandArbiter(lease_seconds=0.2)
    arbiter.accept(CommandOwner.RECOVERY, 1.0)
    arbiter.accept(CommandOwner.RECOVERY, 1.15)
    assert not arbiter.expire(1.21)
    assert arbiter.expire(1.35)
    assert arbiter.owner == CommandOwner.STOPPED


def test_stop_owner_preempts_every_motion_source():
    arbiter = CommandArbiter(lease_seconds=0.2)
    assert arbiter.accept(CommandOwner.RECOVERY, 1.0)[0]
    accepted, changed = arbiter.accept(CommandOwner.STOPPED, 1.01)
    assert accepted and changed
    assert arbiter.owner == CommandOwner.STOPPED
    assert not arbiter.accept(CommandOwner.RECOVERY, 1.02)[0]
    assert not arbiter.expire(1.21)
    assert arbiter.owner == CommandOwner.STOPPED
