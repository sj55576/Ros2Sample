"""Tests for the minimal behavior-tree engine."""

from drone_sim.bt_core import (
    Action,
    Condition,
    format_trace,
    Inverter,
    Selector,
    Sequence,
    Status,
)
import pytest


class Blackboard:
    """Simple attribute bag used as the tick context in tests."""

    def __init__(self, **kwargs):
        self.trace = []
        for key, value in kwargs.items():
            setattr(self, key, value)


def const_action(name, status, log=None):
    """Build an Action returning a fixed status, optionally recording its runs."""
    def effect(bb):
        if log is not None:
            log.append(name)
        return status
    return Action(name, effect)


class TestLeaves:

    def test_condition_true_is_success(self):
        node = Condition('c', lambda bb: True)
        assert node.tick(Blackboard()) == Status.SUCCESS

    def test_condition_false_is_failure(self):
        node = Condition('c', lambda bb: False)
        assert node.tick(Blackboard()) == Status.FAILURE

    def test_action_passes_through_status(self):
        for status in Status:
            assert const_action('a', status).tick(Blackboard()) == status

    def test_action_rejects_non_status_return(self):
        node = Action('bad', lambda bb: True)
        with pytest.raises(TypeError):
            node.tick(Blackboard())


class TestSequence:

    def test_all_success(self):
        node = Sequence('s', [const_action('a', Status.SUCCESS),
                              const_action('b', Status.SUCCESS)])
        assert node.tick(Blackboard()) == Status.SUCCESS

    def test_stops_at_first_failure(self):
        log = []
        node = Sequence('s', [
            const_action('a', Status.SUCCESS, log),
            const_action('b', Status.FAILURE, log),
            const_action('c', Status.SUCCESS, log),
        ])
        assert node.tick(Blackboard()) == Status.FAILURE
        assert log == ['a', 'b']

    def test_stops_at_running(self):
        log = []
        node = Sequence('s', [
            const_action('a', Status.SUCCESS, log),
            const_action('b', Status.RUNNING, log),
            const_action('c', Status.SUCCESS, log),
        ])
        assert node.tick(Blackboard()) == Status.RUNNING
        assert log == ['a', 'b']

    def test_empty_sequence_is_success(self):
        assert Sequence('s', []).tick(Blackboard()) == Status.SUCCESS


class TestSelector:

    def test_first_success_wins(self):
        log = []
        node = Selector('f', [
            const_action('a', Status.FAILURE, log),
            const_action('b', Status.SUCCESS, log),
            const_action('c', Status.SUCCESS, log),
        ])
        assert node.tick(Blackboard()) == Status.SUCCESS
        assert log == ['a', 'b']

    def test_running_pauses_fallback(self):
        node = Selector('f', [
            const_action('a', Status.FAILURE),
            const_action('b', Status.RUNNING),
            const_action('c', Status.SUCCESS),
        ])
        assert node.tick(Blackboard()) == Status.RUNNING

    def test_all_failure(self):
        node = Selector('f', [const_action('a', Status.FAILURE),
                              const_action('b', Status.FAILURE)])
        assert node.tick(Blackboard()) == Status.FAILURE

    def test_empty_selector_is_failure(self):
        assert Selector('f', []).tick(Blackboard()) == Status.FAILURE

    def test_reactive_preemption(self):
        """A higher-priority branch takes over as soon as its condition flips."""
        bb = Blackboard(danger=False)
        log = []
        node = Selector('root', [
            Sequence('high', [
                Condition('danger?', lambda b: b.danger),
                const_action('escape', Status.RUNNING, log),
            ]),
            const_action('work', Status.RUNNING, log),
        ])
        assert node.tick(bb) == Status.RUNNING
        assert log == ['work']
        bb.danger = True
        assert node.tick(bb) == Status.RUNNING
        assert log == ['work', 'escape']


class TestInverter:

    def test_inverts_success_and_failure(self):
        assert Inverter('i', const_action('a', Status.SUCCESS)).tick(Blackboard()) \
            == Status.FAILURE
        assert Inverter('i', const_action('a', Status.FAILURE)).tick(Blackboard()) \
            == Status.SUCCESS

    def test_running_passes_through(self):
        assert Inverter('i', const_action('a', Status.RUNNING)).tick(Blackboard()) \
            == Status.RUNNING


class TestTrace:

    def test_trace_records_visited_nodes_in_tick_order(self):
        bb = Blackboard()
        node = Sequence('s', [
            Condition('c', lambda b: True),
            const_action('a', Status.RUNNING),
        ])
        node.tick(bb)
        assert bb.trace == [
            ('c', Status.SUCCESS),
            ('a', Status.RUNNING),
            ('s', Status.RUNNING),
        ]

    def test_no_trace_attribute_is_fine(self):
        class Bare:
            pass
        assert Condition('c', lambda b: True).tick(Bare()) == Status.SUCCESS

    def test_format_trace(self):
        text = format_trace([('c', Status.SUCCESS), ('a', Status.RUNNING)])
        assert text == 'c:SUCCESS > a:RUNNING'
