import numpy as np
import matplotlib.pyplot as plt
from colorama import Fore

class polynomial:

     # https://www.w3schools.com/python/python_classes.asp

     def __init__(self, x0, dx0, ddx0, x1, dx1, ddx1, t, dt):
         a0 = np.asarray(x0)
         a1 = np.asarray(dx0)
         a2 = np.asarray(np.divide(ddx0, 2))
         x0 = np.asarray(x0)
         dx0 = np.asarray(dx0)
         ddx0 = np.asarray(ddx0)
         x1 = np.asarray(x1)
         dx1 = np.asarray(dx1)
         ddx1 = np.asarray(ddx1)
         self.dt = dt
         self.t = t
         self.trajectory = []



         a3 = (20 * x1 - 20 * x0 - (8 * dx1 + 12 * dx0) * t - (3 * ddx0 -ddx1) * t ** 2) / (2 * t ** 3)
         a4 = (30 * x0 - 30 * x1 + (14 * dx1 + 16 * dx0) * t + (3 * ddx0- 2 * ddx1) * t ** 2) / (2 * t ** 4)
         a5 = (12 * x1 - 12 * x0 - (6 * dx1 + 6 * dx0) * t - (ddx0 -ddx1) * t ** 2) / (2 * t ** 5)


         self.a = [a5, a4, a3, a2, a1, a0]
         self.whole_trajectory_calculate()
         self.velocity_calculate()
         self.acceleration_calculate()

     def trajectory_at_time(self, t):
         a5 = np.asarray(self.a[0])
         a4 = np.asarray(self.a[1])
         a3 = np.asarray(self.a[2])
         a2 = np.asarray(self.a[3])
         a1 = np.asarray(self.a[4])
         a0 = np.asarray(self.a[5])

         y = a5 * t ** 5 + a4 * t ** 4 + a3 * t ** 3 + a2 * t ** 2 + a1 * t + a0

         return y

     def whole_trajectory_calculate(self):
        time_split = np.linspace(0, self.t, int(self.t / self.dt))
        self.trajectory = [0] * len(time_split)

        j = 0
        for i in time_split:
             self.trajectory[j] = self.trajectory_at_time(i)
             j += 1

        return 0
     
     def velocity_calculate(self):
        time_split = np.linspace(0, self.t, int(self.t / self.dt))
        self.velocity = []  # Initialize as an empty list

        j = 0
        for i in time_split:
            if j > 0:
                self.velocity.append((self.trajectory[j] - self.trajectory[j-1]) / self.dt)
            j += 1
        
        return 0
     
     def acceleration_calculate(self):
         time_split = np.linspace(0, self.t, int(self.t / self.dt))
         self.acceleration = []
         print(len(self.velocity))
         self.velocity.append([0, 0])  
               
         j = 0
         for index, i in enumerate(time_split):
            if j > 0:
                print(f"{Fore.BLUE}||{index}||{Fore.GREEN}Vel j : {self.velocity[j]} |{Fore.RED} Vel j-1: {self.velocity[j-1]}")
                self.acceleration.append((self.velocity[j]- self.velocity[j-1]) / self.dt)
            j += 1
         return 0 



if __name__ == "__main__":
    
    test_trajectory = polynomial([0, 7], [0, 0], [0, 0], [10, 3.14], [0, 0], [0, 0], 10, 0.1)

    # Create two subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

    ax1.set_ylabel('Position')
    ax2.set_ylabel('Velocity')
    ax2.set_xlabel('Time')
    ax3.set_ylabel('Acceleration')
    ax3.set_xlabel('Time')
    

    for i in test_trajectory.trajectory:
        ax1.scatter(i[0],i[1])


    for j in test_trajectory.velocity:
        ax2.scatter(j[0], j[1])


    for z in test_trajectory.acceleration:
        ax3.scatter(z[0], z[1])

    plt.show()

    
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
