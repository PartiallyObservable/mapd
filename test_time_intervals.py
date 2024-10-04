import pytest
from time_intervals import *


def test_add_interval():
    intervals = TimeIntervalSet()
    intervals.add_interval(TimeInterval(1, 5))
    intervals.add_interval(TimeInterval(6, 10))
    intervals.add_interval(TimeInterval(0, 2))
    assert intervals.intervals == [
        TimeInterval(0, 2),
        TimeInterval(1, 5),
        TimeInterval(6, 10),
    ]


def test_remove_interval_full_overlap():
    intervals = TimeIntervalSet([TimeInterval(0, 5), TimeInterval(6, 10)])
    intervals.remove_interval(TimeInterval(0, 5))
    assert intervals.intervals == [TimeInterval(6, 10)]


def test_remove_interval_partial_overlap_start():
    intervals = TimeIntervalSet([TimeInterval(0, 10)])
    intervals.remove_interval(TimeInterval(0, 5))
    assert intervals.intervals == [TimeInterval(5, 10)]


def test_remove_interval_partial_overlap_end():
    intervals = TimeIntervalSet([TimeInterval(0, 10)])
    intervals.remove_interval(TimeInterval(5, 10))
    assert intervals.intervals == [TimeInterval(0, 5)]


def test_remove_interval_middle_split():
    intervals = TimeIntervalSet([TimeInterval(0, 10)])
    intervals.remove_interval(TimeInterval(3, 7))
    assert intervals.intervals == [TimeInterval(0, 3), TimeInterval(7, 10)]


def test_remove_no_overlap():
    intervals = TimeIntervalSet([TimeInterval(0, 5), TimeInterval(6, 10)])
    intervals.remove_interval(TimeInterval(11, 15))
    assert intervals.intervals == [TimeInterval(0, 5), TimeInterval(6, 10)]


if __name__ == "__main__":
    pytest.main(["-s", __file__])
