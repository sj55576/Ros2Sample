"""Minimal behavior-tree engine for learning purposes (no ROS imports).

This intentionally implements only the classic core of a behavior tree:

* ``Status``: the tri-state result of a tick (SUCCESS / FAILURE / RUNNING).
* ``BehaviorNode``: base class; every node reports its status per tick.
* ``Sequence``: ticks children left to right, fails/pauses on the first
  non-SUCCESS child (logical AND).
* ``Selector`` (a.k.a. Fallback): ticks children left to right, succeeds/
  pauses on the first non-FAILURE child (logical OR).
* ``Condition`` / ``Action``: leaves wrapping plain callables.
* ``Inverter``: decorator flipping SUCCESS and FAILURE.

Composites here are *reactive* (memory-less): every tick re-evaluates from
the first child. That is what lets a higher-priority branch (e.g. an
emergency landing) preempt a RUNNING lower-priority branch — the key
difference from a finite state machine, where each interrupt needs an
explicit transition from every state.

The blackboard is deliberately untyped: any object shared by all leaves.
If the blackboard has a ``trace`` list attribute, every node appends
``(name, Status)`` to it per tick, so callers can observe the tick path.
"""

from enum import Enum
from typing import Callable, List, Sequence as SequenceType, Tuple


class Status(Enum):
    """Result of ticking a behavior-tree node."""

    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'
    RUNNING = 'RUNNING'


class BehaviorNode:
    """Base class for all behavior-tree nodes."""

    def __init__(self, name: str) -> None:
        self.name = name

    def tick(self, blackboard) -> Status:
        """Evaluate this node, record it in the blackboard trace, and return status."""
        status = self._tick(blackboard)
        trace = getattr(blackboard, 'trace', None)
        if trace is not None:
            trace.append((self.name, status))
        return status

    def _tick(self, blackboard) -> Status:
        """Node-specific tick logic; subclasses must override."""
        raise NotImplementedError


class Sequence(BehaviorNode):
    """Tick children in order; return FAILURE/RUNNING of the first non-SUCCESS child."""

    def __init__(self, name: str, children: SequenceType[BehaviorNode]) -> None:
        super().__init__(name)
        self.children = list(children)

    def _tick(self, blackboard) -> Status:
        for child in self.children:
            status = child.tick(blackboard)
            if status != Status.SUCCESS:
                return status
        return Status.SUCCESS


class Selector(BehaviorNode):
    """Tick children in order; return SUCCESS/RUNNING of the first non-FAILURE child."""

    def __init__(self, name: str, children: SequenceType[BehaviorNode]) -> None:
        super().__init__(name)
        self.children = list(children)

    def _tick(self, blackboard) -> Status:
        for child in self.children:
            status = child.tick(blackboard)
            if status != Status.FAILURE:
                return status
        return Status.FAILURE


class Condition(BehaviorNode):
    """Leaf wrapping a boolean predicate: True -> SUCCESS, False -> FAILURE."""

    def __init__(self, name: str, predicate: Callable[[object], bool]) -> None:
        super().__init__(name)
        self.predicate = predicate

    def _tick(self, blackboard) -> Status:
        return Status.SUCCESS if self.predicate(blackboard) else Status.FAILURE


class Action(BehaviorNode):
    """Leaf wrapping a callable that returns a Status."""

    def __init__(self, name: str, effect: Callable[[object], Status]) -> None:
        super().__init__(name)
        self.effect = effect

    def _tick(self, blackboard) -> Status:
        status = self.effect(blackboard)
        if not isinstance(status, Status):
            raise TypeError(
                f'Action {self.name!r} returned {status!r} instead of a Status'
            )
        return status


class Inverter(BehaviorNode):
    """Decorator inverting SUCCESS <-> FAILURE; RUNNING passes through."""

    def __init__(self, name: str, child: BehaviorNode) -> None:
        super().__init__(name)
        self.child = child

    def _tick(self, blackboard) -> Status:
        status = self.child.tick(blackboard)
        if status == Status.SUCCESS:
            return Status.FAILURE
        if status == Status.FAILURE:
            return Status.SUCCESS
        return status


def format_trace(trace: List[Tuple[str, Status]]) -> str:
    """Render a tick trace as a compact one-line string for logging."""
    return ' > '.join(f'{name}:{status.value}' for name, status in trace)
