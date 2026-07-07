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
        total_duration = time from start to finish
        max_acc = maximum acceleration
        start_pos = starting position
        final_pos = final position
        '''
        self.start_time = start_time
        self.max_acc = max_acc
        self.start_pos = start_pos
        self.final_pos = final_pos
        self.total_duration = total_duration

        # Coefficient used in acceleration calculations
        self.acc_coeff = 36 / 3**(1/2) * max_acc / total_duration**3

    def get_acceleration(self, input_time: float):
        t = input_time - self.start_time
        d = self.total_duration
        if 0 <= t and t < self.total_duration:
            return self.acc_coeff * (t**3 - 3/2 * d * t**2 + 1/2 * d**2 * t)
        return 0
    
    def get_velocity(self, input_time: float):
        t = input_time - self.start_time
        d = self.total_duration
        if 0 <= t and t < self.total_duration:
            return self.acc_coeff * (1/4 * t**4 - 1/2 * d * t**3 + 1/4 * d**2 * t**2)
        return 0
    
    def get_position(self, input_time: float):
        t = input_time - self.start_time
        d = self.total_duration
        if t < 0:
            return self.start_pos
        if t < self.total_duration:
            return self.acc_coeff * (1/20 * t**5 - 1/8 * d * t**4 + 1/12 * d**2 * t**3)
        return self.acc_coeff * 1/120 * d**5

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