from concurrent.futures import Future
from types import SimpleNamespace

from amr_web_bridge.recovery_services import call_empty_service


def client_for(future, available=True):
    removed = []
    return SimpleNamespace(
        wait_for_service=lambda **_kwargs: available,
        call_async=lambda _request: future,
        remove_pending_request=removed.append,
        removed=removed,
    )


def test_empty_response_is_success_without_success_field():
    future = Future()
    future.set_result(object())
    assert call_empty_service(client_for(future), object())[0]


def test_unavailable_service_fails_closed():
    future = Future()
    assert not call_empty_service(client_for(future, False), object())[0]
    assert not future.cancelled()


def test_timeout_removes_pending_request_and_does_not_retry():
    future = Future()
    client = client_for(future)
    ok, detail = call_empty_service(client, object(), timeout_seconds=0.001)
    assert not ok
    assert 'timed out' in detail
    assert client.removed == [future]
    assert future.cancelled()


def test_response_exception_fails_closed():
    future = Future()
    future.set_exception(RuntimeError('disconnected'))
    assert call_empty_service(client_for(future), object()) == (False, 'disconnected')


def test_missing_response_is_not_success():
    future = Future()
    future.set_result(None)
    assert not call_empty_service(client_for(future), object())[0]
