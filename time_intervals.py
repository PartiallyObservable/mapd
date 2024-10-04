from typing import List


class TimeInterval:
    def __init__(self, start, end):
        self.start = start
        self.end = end

    def __repr__(self):
        return f"[{self.start}, {self.end}]"

    def overlaps_with(self, other):
        return self.start < other.end and other.start < self.end

    def __eq__(self, other):
        return self.start == other.start and self.end == other.end


# Class: TimeIntervalSet (formerly SafeIntervalTable)
class TimeIntervalSet:
    def __init__(self, initial_intervals: List["TimeInterval"] = None):
        if initial_intervals is None:
            initial_intervals = []
        self.intervals = sorted(
            initial_intervals.copy(), key=lambda x: x.start
        )  # List of TimeIntervals sorted by start time

    def add_interval(self, interval):
        # Insert the interval in the correct position to keep the list sorted by start time
        index = 0
        while (
            index < len(self.intervals) and self.intervals[index].start < interval.start
        ):
            index += 1
        self.intervals.insert(index, interval)

    def remove_interval(self, interval):
        new_intervals = []
        for current in self.intervals:
            if interval.overlaps_with(current):
                # Case 1: interval completely overlaps current
                if interval.start <= current.start and interval.end >= current.end:
                    continue
                # Case 2: interval partially overlaps from the start
                elif interval.start <= current.start < interval.end < current.end:
                    new_intervals.append(TimeInterval(interval.end, current.end))
                # Case 3: interval partially overlaps from the end
                elif current.start < interval.start < current.end <= interval.end:
                    new_intervals.append(TimeInterval(current.start, interval.start))
                # Case 4: interval is in the middle, splits current into two
                elif current.start < interval.start and interval.end < current.end:
                    new_intervals.append(TimeInterval(current.start, interval.start))
                    new_intervals.append(TimeInterval(interval.end, current.end))
            else:
                # No overlap
                new_intervals.append(current)
        self.intervals = new_intervals

    def __eq__(self, other):
        return self.intervals == other.intervals
