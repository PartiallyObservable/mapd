from time_intervals import *


class Motion:
    def __init__(self, start_state, end_state, time_interval: TimeInterval):
        self.start_state = start_state
        self.end_state = end_state
        self.time_interval = time_interval

    def __repr__(self):
        return f"({self.start_state} -> {self.end_state}), {self.time_interval})"


# Class: Trajectory
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

    def __iter__(self):
        for motion in self.motions:
            yield motion

    def iter_time_interval(self, time_interval: TimeInterval):
        for motion in self.motions:
            if motion.time_interval.overlaps_with(time_interval):
                yield motion

    def __repr__(self):
        rep = [f"{mot}" for mot in self.motions]
        return "\n".join(rep)
