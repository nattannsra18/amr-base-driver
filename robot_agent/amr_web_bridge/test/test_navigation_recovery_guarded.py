from types import SimpleNamespace

from amr_web_bridge.navigation_recovery import NavigationRecoveryRunner


def test_guarded_reverse_accepts_jazzy_trigger_output():
    calls = []

    def run(arguments, **kwargs):
        calls.append((arguments, kwargs))
        return SimpleNamespace(
            returncode=0,
            stdout="std_srvs.srv.Trigger_Response(success=True, message='done')",
            stderr='',
        )

    runner = NavigationRecoveryRunner(run=run)
    succeeded, _ = runner.guarded_reverse()
    assert succeeded
    assert calls[0][0][3] == '/guarded_reverse_escape'


def test_guarded_reverse_propagates_refusal():
    def run(_arguments, **_kwargs):
        return SimpleNamespace(
            returncode=0,
            stdout="std_srvs.srv.Trigger_Response(success=False, message='blocked')",
            stderr='',
        )

    succeeded, output = NavigationRecoveryRunner(run=run).guarded_reverse()
    assert not succeeded
    assert 'blocked' in output


def test_clear_costmaps_allows_dds_service_discovery_time():
    calls = []

    def run(arguments, **kwargs):
        calls.append((arguments, kwargs))
        return SimpleNamespace(returncode=0, stdout='response', stderr='')

    succeeded, detail = NavigationRecoveryRunner(run=run).clear_costmaps()

    assert succeeded
    assert detail == 'Local and global costmaps cleared'
    assert len(calls) == 2
    assert all(call[1]['timeout'] == 15.0 for call in calls)
