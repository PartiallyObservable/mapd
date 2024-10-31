from typing import List, Iterator


class TimeInterval:
    def __init__(self, start, end):
        if start > end:
            raise ValueError(
                f"Start of time interval must be <= end of time interval. Given [{start}, {end}]"
            )
        self.start = start
        self.end = end

    def overlaps(self, other) -> bool:
        return self.start <= other.end and self.end >= other.start

    def contains(self, other) -> bool:
        return self.start <= other.start and self.end >= other.end

    def contained_by(self, other) -> bool:
        return other.contains(self)

    def __eq__(self, other):
        return self.start == other.start and self.end == other.end

    def __repr__(self):
        return f"[{self.start:.3f}, {self.end:.3f}]"


class TimeIntervalSet:
    def __init__(self, initial_intervals: List[TimeInterval] = None):
        if initial_intervals is None:
            initial_intervals = []
        self.intervals = sorted(initial_intervals.copy(), key=lambda x: x.start)

    def add_interval(self, interval):
        new_intervals = []
        added = False

        for current in self.intervals:
            if current.end < interval.start:
                new_intervals.append(current)
            elif interval.end < current.start:
                if not added:
                    new_intervals.append(interval)
                    added = True
                new_intervals.append(current)
            else:
                # Merge overlapping intervals
                interval = TimeInterval(
                    min(interval.start, current.start), max(interval.end, current.end)
                )

        if not added:
            new_intervals.append(interval)

        self.intervals = new_intervals

    def remove_interval(self, interval: TimeInterval):
        new_intervals = []
        for current in self.intervals:
            if interval.overlaps(current):
                if interval.start <= current.start and interval.end >= current.end:
                    continue
                elif interval.start <= current.start < interval.end < current.end:
                    new_intervals.append(TimeInterval(interval.end, current.end))
                elif current.start < interval.start < current.end <= interval.end:
                    new_intervals.append(TimeInterval(current.start, interval.start))
                elif current.start < interval.start and interval.end < current.end:
                    new_intervals.append(TimeInterval(current.start, interval.start))
                    new_intervals.append(TimeInterval(interval.end, current.end))
            else:
                new_intervals.append(current)
        self.intervals = new_intervals

    def contains(self, interval: TimeInterval):
        for current in self.intervals:
            if current.contains(interval):
                return True
        return False
    
    def __iter__(self) -> Iterator[TimeInterval]:
        return iter(self.intervals)

    def __eq__(self, other):
        if not isinstance(other, TimeIntervalSet):
            return False
        return self.intervals == other.intervals

    def __repr__(self):
        return f"({self.intervals})"
