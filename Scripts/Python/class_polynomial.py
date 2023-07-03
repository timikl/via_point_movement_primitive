import enum
import numpy as np
class Polynomial:

    # https://www.w3schools.com/python/python_classes.asp

    def __init__(self, x0, dx0, ddx0, x1, dx1, ddx1, t, dt):

        self.x0 = np.asarray(x0)
        self.dx0 = np.asarray(dx0)
        self.ddx0 = np.asarray(ddx0)
        self.x1 = np.asarray(x1)
        self.dx1 = np.asarray(dx1)
        self.ddx1 = np.asarray(ddx1)
        self.dt = dt
        self.t = t
        self.trajectory = []
        self.time_split = np.linspace(0, self.t, int(self.t / self.dt))

        # Polynomial parameter A
        a0 = np.asarray(x0)
        a1 = np.asarray(dx0)
        a2 = np.asarray(np.divide(ddx0, 2))
        a3 = (20 * self.x1 - 20 * self.x0 - (8 * self.dx1 + 12 * self.dx0) * t - (3 * self.ddx0 - self.ddx1) * t ** 2) / (2 * t ** 3)
        a4 = (30 * self.x0 - 30 * self.x1 + (14 * self.dx1 + 16 * self.dx0) * t + (3 * self.ddx0 - 2 * self.ddx1) * t ** 2) / (2 * t ** 4)
        a5 = (12 * self.x1 - 12 * self.x0 - (6 * self.dx1 + 6 * self.dx0) * t - (self.ddx0 - self.ddx1) * t ** 2) / (2 * t ** 5)

        self.a = [a5, a4, a3, a2, a1, a0]

    def trajectory_at_time(self, t):
        a5 = np.asarray(self.a[0])
        a4 = np.asarray(self.a[1])
        a3 = np.asarray(self.a[2])
        a2 = np.asarray(self.a[3])
        a1 = np.asarray(self.a[4])
        a0 = np.asarray(self.a[5])

        y = a5 * t ** 5 + a4 * t ** 4 + a3 * t ** 3 + a2 * t ** 2 + a1 * t + a0
        # dy = a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4
        # ddy = 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2 + 20 * a5 * t **3
        return y

    def whole_trajectory_calculate(self):

        self.trajectory = [0] * len(self.time_split)

        j = 0
        for i in self.time_split:
            self.trajectory[j] = self.trajectory_at_time(i)
            j += 1

        return 0

    def velocity_calculate(self):
        self.velocity = [self.dx0]  # Initialize as an empty list

        j = 0
        for i in enumerate(self.time_split):
            if j > 0:
                self.velocity.append((self.trajectory[j] - self.trajectory[j - 1]) / self.dt)
            j += 1

        return 0

    def acceleration_calculate(self):
        self.acceleration = [self.ddx0]

        j = 0
        for i in enumerate(self.time_split):
            if j > 0:
                self.acceleration.append((self.velocity[j] - self.velocity[j - 1]) / self.dt)

            j += 1

        return 0

    def jerk_calculate(self):
        self.jerk = []

        j = 0
        for i in enumerate(self.time_split):
            if j > 0:
                self.jerk.append((self.acceleration[j] - self.acceleration[j - 1]) / self.dt)
                # print(f"{Fore.LIGHTGREEN_EX}Jerk: {self.jerk}")
            j += 1

        return 0


"""
        num_steps = int(t/dt + 1)
        # Create arrays of zeros
        y = np.zeros((2, num_steps), dtype=np.int32)
        dy = np.zeros((2, num_steps), dtype=np.int32)
        ddy = np.zeros((2, num_steps), dtype=np.int32)

        for i in range(len(y)):
              i_t = (i-1)*dt;
              y(i) = a(1) * i_t^5 + a(2) * i_t^4 + a(3) * i_t^3 + a(4) * i_t^2 + a(5) * i_t + a(6);
              dy(i) = 5 * a(1) * i_t^4 + 4 * a(2) * i_t^3 + 3 * a(3) * i_t^2 + 2 * a(4) * i_t + a(5);
              ddy(i) = 20 * a(1) * i_t^3 + 12 * a(2) * i_t^2 + 6 * a(3) * i_t + 2 * a(4);
"""
