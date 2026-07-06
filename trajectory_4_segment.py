# Graph the trajectories of motors using polynomial smoothing.

import matplotlib.pyplot as plt

class Trajectory:
    def __init__(self,
                 start_time: float,
                 total_duration: float,
                 max_acc: float,
                 start_pos: float,
                 final_pos: float):
        '''
        start_time = time acceleration begins
        total_duration = time from acceleration start to last position
        max_acc = maximum acceleration
        start_pos = starting position
        final_pos = final position
        '''
        self.max_acc = max_acc
        self.start_pos = start_pos
        self.final_pos = final_pos

        self.tt = total_duration
        self.t0 = start_time
        self.t1 = start_time + total_duration / 2
        self.t2 = start_time + total_duration

        # Coefficient used in acceleration calculations
        self.acc_coeff = 16 * max_acc / total_duration**2

    def get_acceleration(self, t: float):
        if t < self.t0:
            return 0
        if t < self.t1:
            return - self.acc_coeff * (t - self.t0) * (t - self.t0 - self.tt / 2)
        if t < self.t2:
            return self.acc_coeff * (t - self.t0 - self.tt / 2) * (t - self.t0 - self.tt)
        return 0
    
    def get_velocity(self, t: float):
        if t < self.t0:
            return 0
        if t < self.t1:
            return - self.acc_coeff * (
                1/3 * t**3
                - (self.t0 + 1/4 * self.tt) * t**2
                + (self.t0**2 + 1/2 * self.t0 * self.tt) * t
                - (1/3 * self.t0**3 + 1/4 * self.t0**2 * self.tt)
            )
        elif t < self.t2:
            return self.acc_coeff * (
                1/3 * t**3
                - (self.t0 + 3/4 * self.tt) * t**2
                + (self.t0**2 + 3/2 * self.t0 * self.tt + 1/2 * self.tt**2) * t
                - (
                    1/3 * self.t0**3
                    + 3/4 * self.t0**2 * self.tt
                    + 1/2 * self.t0 * self.tt**2
                    + 1/12 * self.tt**3
                )
            )
        return 0
    
    def get_position(self, t: float):
        if t < self.t0:
            return self.start_pos
        if t < self.t1:
            return self.start_pos - self.acc_coeff * (
                1/12 * t**4
                - (1/3 * self.t0 + 1/12 * self.tt) * t**3
                + (1/2 * self.t0**2 + 1/4 * self.t0 * self.tt) * t**2
                - (1/3 * self.t0**3 + 1/4 * self.t0**2 * self.tt) * t
                + 1/12 * self.t0**3 * (self.t0 + self.tt)
            )
        if t < self.t2:
            return self.start_pos + self.acc_coeff * (
                1/12 * t**4
                - (1/3 * self.t0 + 1/4 * self.tt) * t**3
                + (1/2 * self.t0**2 + 3/4 * self.t0 * self.tt + 1/4 * self.tt**2) * t**2
                - (
                    1/3 * self.t0**3
                    + 3/4 * self.t0**2 * self.tt
                    + 1/2 * self.t0 * self.tt**2
                    + 1/12 * self.tt**3
                ) * t
                + 1/96 * self.tt**4
                + 1/12 * self.t0 * self.tt**3
                + 1/4 * self.t0**2 * self.tt**2
                + 1/4 * self.t0**3 * self.tt
                + 1/12 * self.t0**4
            )
        return self.final_pos

TOTAL_TIME = 10
DATA_POINT_COUNT = TOTAL_TIME * 10

# Horizontal axis
times = [t/TOTAL_TIME for t in range(0, DATA_POINT_COUNT)]

trajectory = Trajectory(
    start_time=1,
    total_duration=5,
    max_acc=10,
    start_pos=0.5,
    final_pos=2.5
)

accelerations = [trajectory.get_acceleration(t) for t in times]
velocities = [trajectory.get_velocity(t) for t in times]
positions = [trajectory.get_position(t) for t in times]

plt.plot(times, accelerations)
plt.plot(times, velocities)
plt.plot(times, positions)
plt.show()