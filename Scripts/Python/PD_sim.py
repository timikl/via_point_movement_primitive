import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Parameters
tau = 1.0         # Time constant
alpha_z = 2.0     # PD gain
beta_z = 1.0      # Scaling factor
g = 1.0           # Goal position

# Initial conditions
y0 = 0.0          # Initial position
y0_dot = 0.0      # Initial velocity
t_span = (0, 10)  # Time span for the simulation
t_eval = np.linspace(*t_span, 500)  # Time points to evaluate

# Define the system of ODEs
def pd_controller(t, y):
    y1, y2 = y  # y1 is position, y2 is velocity
    y1_dot = y2
    y2_dot = (alpha_z * beta_z * (g - y1) - alpha_z * y2) / tau
    return [y1_dot, y2_dot]

# Solve the ODE
solution = solve_ivp(
    pd_controller, t_span, [y0, y0_dot], t_eval=t_eval, method='RK45'
)

# Extract results
time = solution.t
position = solution.y[0]

# Plot the trajectory
plt.figure(figsize=(10, 6))
plt.plot(time, position, label="Trajectory y(t)", linewidth=2)
plt.axhline(y=g, color='r', linestyle='--', label="Goal position (g)")
plt.title("PD Controller Trajectory")
plt.xlabel("Time (t)")
plt.ylabel("Position y(t)")
plt.legend()
plt.grid()
plt.show()
