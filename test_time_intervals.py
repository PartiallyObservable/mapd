import pytest
from time_intervals import *


def test_add_interval():
    # Test adding intervals that are non-overlapping but not sorted
    intervals = TimeIntervalSet()
    intervals.add_interval(TimeInterval(2, 5))
    intervals.add_interval(TimeInterval(6, 10))
    intervals.add_interval(TimeInterval(0, 1))
    assert intervals.intervals == [
        TimeInterval(0, 1),
        TimeInterval(2, 5),
        TimeInterval(6, 10),
    ]


def test_add_interval_with_merge():
    # Test adding overlapping intervals and merging them
    intervals = TimeIntervalSet()
    intervals.add_interval(TimeInterval(1, 5))
    intervals.add_interval(TimeInterval(4, 8))  # Overlaps with previous
    assert intervals.intervals == [TimeInterval(1, 8)]
    intervals.add_interval(TimeInterval(0, 2))  # Overlaps with first part of [1, 8]
    assert intervals.intervals == [TimeInterval(0, 8)]
    intervals.add_interval(TimeInterval(9, 12))  # Non-overlapping
    assert intervals.intervals == [TimeInterval(0, 8), TimeInterval(9, 12)]


def test_remove_interval_full_overlap():
    # Test removing an interval that completely overlaps an existing interval
    intervals = TimeIntervalSet([TimeInterval(0, 5), TimeInterval(6, 10)])
    intervals.remove_interval(TimeInterval(0, 5))
    assert intervals.intervals == [TimeInterval(6, 10)]


def test_remove_interval_partial_overlap_start():
    # Test removing a partial overlap from the start of an interval
    intervals = TimeIntervalSet([TimeInterval(0, 10)])
    intervals.remove_interval(TimeInterval(0, 5))
    assert intervals.intervals == [TimeInterval(5, 10)]


def test_remove_interval_partial_overlap_end():
    # Test removing a partial overlap from the end of an interval
    intervals = TimeIntervalSet([TimeInterval(0, 10)])
    intervals.remove_interval(TimeInterval(5, 10))
    assert intervals.intervals == [TimeInterval(0, 5)]


def test_remove_interval_middle_split():
    # Test removing an interval that splits an existing interval into two parts
    intervals = TimeIntervalSet([TimeInterval(0, 10)])
    intervals.remove_interval(TimeInterval(3, 7))
    assert intervals.intervals == [TimeInterval(0, 3), TimeInterval(7, 10)]


def test_remove_no_overlap():
    # Test removing an interval that has no overlap with existing intervals
    intervals = TimeIntervalSet([TimeInterval(0, 5), TimeInterval(6, 10)])
    intervals.remove_interval(TimeInterval(11, 15))
    assert intervals.intervals == [TimeInterval(0, 5), TimeInterval(6, 10)]


def test_remove_multiple_overlapping_intervals():
    # Test removing an interval that overlaps with multiple intervals
    intervals = TimeIntervalSet(
        [TimeInterval(0, 5), TimeInterval(6, 10), TimeInterval(11, 15)]
    )
    intervals.remove_interval(TimeInterval(4, 12))
    assert intervals.intervals == [TimeInterval(0, 4), TimeInterval(12, 15)]


def test_remove_interval_equal_to_existing():
    # Test removing an interval that is equal to an existing interval
    intervals = TimeIntervalSet([TimeInterval(0, 5)])
    intervals.remove_interval(TimeInterval(0, 5))
    assert intervals.intervals == []


def test_add_invalid_interval():
    # Test adding an invalid interval (end < start)
    intervals = TimeIntervalSet()
    with pytest.raises(ValueError):
        intervals.add_interval(TimeInterval(5, 1))  # Invalid interval


def test_contains_interval():
    # Test if a TimeIntervalSet contains a given interval
    intervals = TimeIntervalSet([TimeInterval(0, 10), TimeInterval(15, 20)])
    assert intervals.contains(TimeInterval(5, 7)) is True
    assert intervals.contains(TimeInterval(0, 10)) is True
    assert intervals.contains(TimeInterval(10, 15)) is False  # Gap between intervals


if __name__ == "__main__":
    pytest.main(["-s", __file__])
