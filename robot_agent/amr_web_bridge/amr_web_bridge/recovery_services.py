"""Bounded worker-thread calls using the agent's existing ROS participant."""

import threading


def call_service(client, request, *, timeout_seconds=2.0):
    """Return (response, detail); the ROS executor must keep spinning.

    Do not call from a ROS callback on the single-threaded executor. A timeout
    does not retract a request already delivered to the server; callers must
    fail closed, never retry a motion command using this helper.
    """
    future = None
    try:
        if not client.wait_for_service(timeout_sec=0.25):
            return None, 'service is unavailable'
        finished = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: finished.set())
        if not finished.wait(timeout_seconds):
            client.remove_pending_request(future)
            future.cancel()
            return None, f'service response timed out after {timeout_seconds:.1f}s'
        response = future.result()
        if response is None:
            return None, 'service returned no response'
        return response, 'service completed'
    except Exception as error:
        return None, str(error)


def call_empty_service(client, request, *, timeout_seconds=2.0):
    response, detail = call_service(client, request, timeout_seconds=timeout_seconds)
    return response is not None, detail
