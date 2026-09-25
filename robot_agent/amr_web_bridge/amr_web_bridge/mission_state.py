"""Small, ROS-independent mission command admission state."""

from __future__ import annotations

from collections import OrderedDict


class MissionCommandState:
    """Bound command de-duplication without coupling it to WebSockets or maps."""

    def __init__(self, processed_limit: int = 512):
        self.pending_ids: set[str] = set()
        self.processed_ids: OrderedDict[str, None] = OrderedDict()
        self.processed_limit = max(1, int(processed_limit))

    def admit(self, command_id: str, *, active_id: str | None = None) -> bool:
        """Return true exactly once for a new command identifier."""
        if (
            command_id == active_id
            or command_id in self.pending_ids
            or command_id in self.processed_ids
        ):
            return False
        self.pending_ids.add(command_id)
        self.processed_ids[command_id] = None
        while len(self.processed_ids) > self.processed_limit:
            self.processed_ids.popitem(last=False)
        return True

    def remove_pending(self, command_id: str) -> None:
        self.pending_ids.discard(command_id)

    def clear_pending(self) -> None:
        self.pending_ids.clear()
