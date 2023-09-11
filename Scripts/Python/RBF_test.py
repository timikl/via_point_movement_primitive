import numpy as np
import matplotlib.pyplot as plt
from Trj2RBF import Trj2RBF

# Set the time step and time range
dt = 0.01
t = np.arange(0, 1+dt, dt)

# Compute y based on the given formula
y = np.sin(t * 2 * np.pi) * np.cos(6 * t * 2 * np.pi)

# Plot y in red
plt.plot(y, 'r')

# Call the Trj2RBF function (you need to have this function defined or imported)
RBF, yRBF = Trj2RBF(y, dt, 20)

# Plot yRBF in blue with dotted lines
plt.plot(yRBF, 'b:')

# Display the plot
plt.show()
