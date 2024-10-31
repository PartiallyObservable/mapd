from typing import Iterator, Generator
from common.time_intervals import *
from common.graph import Graph


# Motion represents the transition between two states on a graph over a time interval
# It can correspond to a graph-edge if the nodes are different,
#   or waiting at a graph node if nodes are the same
class Motion:
    def __init__(
        self,
        start_state: Graph.Node,
        end_state: Graph.Node,
        time_interval: TimeInterval,
    ):
        self.start_state = start_state
        self.end_state = end_state
        self.time_interval = time_interval

    def is_transition(self) -> bool:
        return self.start_state != self.end_state

    def is_waiting(self) -> bool:
        return self.start_state == self.end_state

    def __repr__(self):
        return f"({self.start_state} -> {self.end_state}), {self.time_interval})"


# Trajectory is a sequence of contiguous motions
class Trajectory:
    def __init__(self):
        self.motions = []

    def add_motion(self, motion: Motion):
        if not self.motions:
            self.motions.append(motion)
        else:
            last_motion = self.motions[-1]
            if (
                last_motion.end_state == motion.start_state
                and last_motion.time_interval.end == motion.time_interval.start
            ):
                self.motions.append(motion)
            else:
                raise ValueError("Motion is not contiguous with the current trajectory")

    def __iter__(self) -> Iterator[Motion]:
        return iter(self.motions)

    def iter_time_interval(self, time_interval: TimeInterval) -> Generator[Motion, None, None]:
        for motion in self.motions:
            if motion.time_interval.overlaps_with(time_interval):
                yield motion

    def __repr__(self):
        rep = [f"{mot}" for mot in self.motions]
        return "\n".join(rep)
