"""A small, bounded, thread-safe result cache.

Identical requests are cheap to repeat — the dashboard's controller comparison asks for the
same LQR run the user has already seen, and visitors reload the page — so a completed run is
kept keyed on the hash of its *validated* request. Because every field is bounded and the
scenario is a pure function of those fields, an identical key really does mean an identical
simulation.

The cache is deliberately small and in-process: it protects the CPU, it is not a database.
"""

from __future__ import annotations

import threading
from collections import OrderedDict
from typing import Any


class LruCache:
    """Least-recently-used cache with a fixed capacity.

    ``capacity <= 0`` disables caching entirely, which keeps the call sites branch-free.
    """

    def __init__(self, capacity: int) -> None:
        self._capacity = max(0, int(capacity))
        self._entries: "OrderedDict[str, Any]" = OrderedDict()
        self._lock = threading.Lock()
        self._hits = 0
        self._misses = 0

    def get(self, key: str) -> Any | None:
        if self._capacity == 0:
            return None
        with self._lock:
            if key not in self._entries:
                self._misses += 1
                return None
            self._entries.move_to_end(key)
            self._hits += 1
            return self._entries[key]

    def put(self, key: str, value: Any) -> None:
        if self._capacity == 0:
            return
        with self._lock:
            self._entries[key] = value
            self._entries.move_to_end(key)
            while len(self._entries) > self._capacity:
                self._entries.popitem(last=False)

    def clear(self) -> None:
        with self._lock:
            self._entries.clear()

    def stats(self) -> dict:
        with self._lock:
            return {
                "capacity": self._capacity,
                "entries": len(self._entries),
                "hits": self._hits,
                "misses": self._misses,
            }
