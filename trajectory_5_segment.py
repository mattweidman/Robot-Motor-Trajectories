# Graph the trajectories of motors using polynomial smoothing.

import matplotlib.pyplot as plt

class Trajectory:
    def __init__(self,
                 start_time: float,
                 acc_duration: float,
                 vel_duration: float,
                 max_acc: float):
        '''
        start_time = time acceleration begins
        acc_duration = time acceleration lasts (both first and last interval)
        vel_duration = time the constant velocity interval lasts
        max_acc = maximum acceleration
        '''
        self.acc_duration = acc_duration
        self.vel_duration = vel_duration
        self.max_acc = max_acc

        self.t0 = start_time
        self.t1 = self.t0 + acc_duration # t1 = time first acceleration interval ends
        self.t2 = self.t1 + vel_duration # t2 = time second acceleration interval begins
        self.t3 = self.t2 + acc_duration # t3 = time entire trajectory ends

        self.acc_coefficient = 4 * max_acc / acc_duration**2

    def get_acceleration(self, t: float):
        if self.t0 < t and t < self.t1:
            return - self.acc_coefficient * (t**2 - (self.t0 + self.t1) * t + self.t0 * self.t1)
        if self.t2 < t and t < self.t3:
            return self.acc_coefficient * (t**2 - (self.t2 + self.t3) * t + self.t2 * self.t3)
        return 0

TOTAL_TIME = 10
DATA_POINT_COUNT = TOTAL_TIME * 10

# Horizontal axis
times = [t/TOTAL_TIME for t in range(0, DATA_POINT_COUNT)]

trajectory = Trajectory(
    start_time=1,
    acc_duration=2,
    vel_duration=3,
    max_acc=10
)

# Acceleration
accelerations = [trajectory.get_acceleration(t) for t in times]

plt.plot(times, accelerations)
plt.show()