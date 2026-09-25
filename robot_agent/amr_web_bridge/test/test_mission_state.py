from amr_web_bridge.mission_state import MissionCommandState


def test_command_is_admitted_only_once_across_pending_and_processed_states():
    state = MissionCommandState()

    assert state.admit('command-1')
    assert not state.admit('command-1')
    state.remove_pending('command-1')
    assert not state.admit('command-1')


def test_active_command_is_rejected_without_mutating_history():
    state = MissionCommandState()

    assert not state.admit('command-active', active_id='command-active')
    assert state.pending_ids == set()
    assert list(state.processed_ids) == []


def test_processed_history_is_bounded_but_pending_ids_remain_explicit():
    state = MissionCommandState(processed_limit=2)

    assert state.admit('command-1')
    assert state.admit('command-2')
    assert state.admit('command-3')

    assert list(state.processed_ids) == ['command-2', 'command-3']
    assert state.pending_ids == {'command-1', 'command-2', 'command-3'}
    state.clear_pending()
    assert state.pending_ids == set()
