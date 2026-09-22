from amr_base_driver.cmd_vel_passthrough import EscapePriorityGate


def test_escape_lease_blocks_normal_commands_then_expires():
    gate = EscapePriorityGate(lease_seconds=0.2)
    assert gate.accept_normal(1.0)
    gate.accept_escape(1.0)
    assert not gate.accept_normal(1.1)
    assert not gate.expire(1.19)
    assert gate.expire(1.20)
    assert gate.accept_normal(1.20)


def test_escape_stream_renews_lease():
    gate = EscapePriorityGate(lease_seconds=0.2)
    gate.accept_escape(1.0)
    gate.accept_escape(1.15)
    assert not gate.expire(1.21)
    assert gate.expire(1.35)
