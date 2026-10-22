import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch.actions import LogInfo


LAUNCH_FILE = (
    Path(__file__).resolve().parents[1] / 'launch' / 'navigation.launch.py'
)


def load_navigation_launch():
    spec = importlib.util.spec_from_file_location(
        'navigation_launch_under_test', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_readiness_success_releases_navigation_processes():
    module = load_navigation_launch()
    navigation_processes = object()

    actions = module.navigation_after_readiness_exit(
        SimpleNamespace(returncode=0), navigation_processes)

    assert isinstance(actions[0], LogInfo)
    assert actions[1] is navigation_processes


def test_readiness_failure_keeps_navigation_processes_stopped():
    module = load_navigation_launch()
    navigation_processes = object()

    actions = module.navigation_after_readiness_exit(
        SimpleNamespace(returncode=1), navigation_processes)

    assert len(actions) == 1
    assert isinstance(actions[0], LogInfo)
    assert navigation_processes not in actions


def test_readiness_signal_exit_keeps_navigation_processes_stopped():
    module = load_navigation_launch()
    navigation_processes = object()

    actions = module.navigation_after_readiness_exit(
        SimpleNamespace(returncode=-9), navigation_processes)

    assert len(actions) == 1
    assert isinstance(actions[0], LogInfo)
    assert navigation_processes not in actions
