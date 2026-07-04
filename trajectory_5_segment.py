# Graph the trajectories of motors using polynomial smoothing.

import matplotlib.pyplot as plt

class Trajectory:
    def __init__(self,
                 start_time: float,
                 acc_duration: float,
                 vel_duration: float,
                 max_acc: float,
                 start_pos: float,
                 final_pos: float):
        '''
        start_time = time acceleration begins
        acc_duration = time acceleration lasts (both first and last interval)
        vel_duration = time the constant velocity interval lasts
        max_acc = maximum acceleration
        start_pos = starting position
        final_pos = final position
        '''
        self.acc_duration = acc_duration
        self.vel_duration = vel_duration
        self.max_acc = max_acc
        self.start_pos = start_pos
        self.final_pos = final_pos

        self.t0 = start_time
        self.t1 = self.t0 + acc_duration # t1 = time first acceleration interval ends
        self.t2 = self.t1 + vel_duration # t2 = time second acceleration interval begins
        self.t3 = self.t2 + acc_duration # t3 = time entire trajectory ends

        # Coefficient used in acceleration calculations
        self.acc_coeff = 4 * max_acc / acc_duration**2

    def get_acceleration(self, t: float):
        if self.t0 <= t and t < self.t1:
            return -self.acc_coeff * (t**2 - (self.t0 + self.t1) * t + self.t0 * self.t1)
        if self.t2 <= t and t < self.t3:
            return self.acc_coeff * (t**2 - (self.t2 + self.t3) * t + self.t2 * self.t3)
        return 0
    
    def get_velocity(self, t: float):
        if self.t0 <= t and t < self.t1:
            return -self.acc_coeff * (
                1/3 * t**3
                - 1/2 * (self.t0 + self.t1) * t**2
                + self.t0 * self.t1 * t
                - 1/6 * self.t0**2 * (3 * self.t1 - self.t0))
        if self.t1 <= t and t < self.t2:
            return -self.acc_coeff / 6 * (
                self.t0**3 - self.t1**3
                + 3 * self.t0 * self.t1 * (self.t1 - self.t0))
        if self.t2 <= t and t < self.t3:
            return self.acc_coeff * (
                1/3 * t**3
                - 1/2 * (self.t2 + self.t3) * t**2
                + self.t2 * self.t3 * t
                + 1/6 * self.t3**3
                - 1/2 * self.t2 * self.t3**2)
        return 0
    
    def get_position(self, t: float):
        if t < self.t0:
            return self.start_pos
        if self.t0 <= t and t < self.t1:
            return self.start_pos - self.acc_coeff / 12 * (
                t**4
                - 2 * (self.t0 + self.t1) * t**3
                + 6 * self.t0 * self.t1 * t**2
                - 2 * self.t0**2 * (3 * self.t1 - self.t0) * t
                - self.t0**3 * (self.t0 - 2 * self.t1)
            )
        if self.t1 <= t and t < self.t2:
            return self.start_pos - self.acc_coeff / 12 * (
                (
                    2 * self.t0**3 - 2 * self.t1**3
                    + 6 * self.t0 * self.t1 * (self.t1 - self.t0)
                ) * t
                + self.t1**4
                - 2 * self.t0 * self.t1**3
                + 2 * self.t0**3 * self.t1
                - self.t0**4
            )
        if self.t2 <= t and t < self.t3:
            c = self.acc_coeff / 12 * (
                self.t2**4
                - 4 * self.t2**3 * self.t3
                + 6 * self.t2**2 * self.t3**2
                - 2 * self.t2 * self.t3**3
                - 2 * self.t0**3 * self.t2
                + 2 * self.t1**3 * self.t2
                + 6 * self.t0**2 * self.t1 * self.t2
                - 6 * self.t0 * self.t1**2 * self.t2
                - self.t1**4
                + 2 * self.t0 * self.t1**3
                - 2 * self.t0**3 * self.t1
                + self.t0**4
            )
            return c + self.start_pos + self.acc_coeff / 12 * (
                t**4
                - 2 * (self.t2 + self.t3) * t**3
                + 6 * self.t2 * self.t3 * t**2
                + 2 * self.t3**3 * t
                - 6 * self.t2 * self.t3**2 * t
            )
        return self.final_pos

TOTAL_TIME = 10
DATA_POINT_COUNT = TOTAL_TIME * 10

# Horizontal axis
times = [t/TOTAL_TIME for t in range(0, DATA_POINT_COUNT)]

trajectory = Trajectory(
    start_time=1,
    acc_duration=2,
    vel_duration=3,
    max_acc=10,
    start_pos=0.5,
    final_pos=2.5
)

accelerations = [trajectory.get_acceleration(t) for t in times]
velocities = [trajectory.get_velocity(t) for t in times]
positions = [trajectory.get_position(t) for t in times]

plt.plot(times, positions)
plt.show()