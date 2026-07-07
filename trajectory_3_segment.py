# Graph the trajectories of motors using polynomial smoothing.

import matplotlib.pyplot as plt

class Trajectory:
    def __init__(self,
                 start_time: float,
                 start_pos: float,
                 final_pos: float,
                 duration: float):
        self.start_time = start_time
        self.start_pos = start_pos
        self.final_pos = final_pos
        self.duration = duration

        # Coefficient used in most acceleration, velocity, and position calculations
        self.coeff = 120 * (final_pos - start_pos) / self.duration**5

    def get_acceleration(self, input_time: float):
        t = input_time - self.start_time
        d = self.duration
        if 0 <= t and t < self.duration:
            return self.coeff * (t**3 - 3/2 * d * t**2 + 1/2 * d**2 * t)
        return 0
    
    def get_velocity(self, input_time: float):
        t = input_time - self.start_time
        d = self.duration
        if 0 <= t and t < self.duration:
            return self.coeff * (1/4 * t**4 - 1/2 * d * t**3 + 1/4 * d**2 * t**2)
        return 0
    
    def get_position(self, input_time: float):
        t = input_time - self.start_time
        d = self.duration
        if t < 0:
            return self.start_pos
        if t < self.duration:
            return (
                self.coeff * (1/20 * t**5 - 1/8 * d * t**4 + 1/12 * d**2 * t**3)
                + self.start_pos
            )
        return self.coeff * 1/120 * d**5 + self.start_pos
    
    def get_max_velocity(self):
        return 15/8 * (self.final_pos - self.start_pos) / self.duration
    
    def get_max_acceleration(self):
        return 10/3 * 3**(1/2) * (self.final_pos - self.start_pos) / self.duration**2
    
    def get_midpoint_time(self):
        return self.start_time + self.duration / 2
    
    def get_acceleration_extrema_times(self):
        '''
        Returns the times acceleration peaks in the first half and hits a
        minimum in the second half.
        '''
        return (
            self.duration * (1/2 - 3**(1/2)/6) + self.start_time,
            self.duration * (1/2 + 3**(1/2)/6) + self.start_time
        )
    
    def get_time_given_position(self, pos: float):
        '''
        Uses Newton's method to compute the time that a position corresponds to.
        The position must be between start_pos and final_pos.
        '''
        min_pos = min(self.start_pos, self.final_pos)
        max_pos = max(self.start_pos, self.final_pos)
        if pos < min_pos or pos > max_pos:
            raise Exception("Provided position must be between initial and final position")
        
        guess = self.get_midpoint_time()
        for i in range(6):
            guess_pos = self.get_position(guess) - pos
            guess_vel = self.get_velocity(guess)
            if abs(guess_vel) < 0.00001:
                return guess
            guess -= guess_pos / guess_vel
        
        return guess

def create_trajectory_from_max_acceleration(
        start_time: float,
        start_pos: float,
        final_pos: float,
        max_acc: float):
    duration = (10/3 * 3**(1/2) * abs(final_pos - start_pos) / max_acc)**(1/2)
    return Trajectory(start_time, start_pos, final_pos, duration)

def create_trajectory_from_max_velocity(
        start_time: float,
        start_pos: float,
        final_pos: float,
        max_vel: float):
    duration = 15/8 * abs(final_pos - start_pos) / max_vel
    return Trajectory(start_time, start_pos, final_pos, duration)

TOTAL_TIME = 10
DATA_POINT_COUNT = TOTAL_TIME * 10

# Horizontal axis
times = [t/TOTAL_TIME for t in range(0, DATA_POINT_COUNT)]

trajectory = Trajectory(
    start_time=1,
    start_pos=0.5,
    final_pos=-2.5,
    duration=5,
)

accelerations = [trajectory.get_acceleration(t) for t in times]
velocities = [trajectory.get_velocity(t) for t in times]
positions = [trajectory.get_position(t) for t in times]

plt.plot(times, accelerations)
plt.plot(times, velocities)
plt.plot(times, positions)

# Display max velocity
plt.plot(
    trajectory.get_midpoint_time(),
    trajectory.get_max_velocity(),
    "ro", markersize=5)

# Display max and min acceleration
plt.plot(
    trajectory.get_acceleration_extrema_times()[0],
    trajectory.get_max_acceleration(),
    "go", markersize=5)
plt.plot(
    trajectory.get_acceleration_extrema_times()[1],
    -trajectory.get_max_acceleration(),
    "go", markersize=5)

# Show a point that was found with Newton's method
sample_pos = 0.5
sample_time = trajectory.get_time_given_position(sample_pos)
plt.plot(sample_time, sample_pos, "bo", markersize=5)

plt.show()