import numpy as np
import matplotlib.pyplot as plt
from Trj2RBF import Trj2RBF

if __name__ == "__main__":
    # Set the time step and time range
    dt = 0.01
    t = np.arange(0, 1+dt, dt)

    # Compute y based on the given formula
    y = np.sin(t * 2 * np.pi) * np.cos(6 * t * 2 * np.pi)
    y = y.reshape(-1, 1)

    # Plot y in red
    plt.plot(y, 'r', label="y")

    # Call the Trj2RBF function (you need to have this function defined or imported)
    RBF, yRBF = Trj2RBF(y, dt, 60)

    # Plot yRBF in blue with dotted lines
    plt.plot(yRBF, 'b:', label="RBF")
    plt.xlabel('x') 
    plt.ylabel('t') 
    plt.title("RBF and y functions")
    plt.legend()

    print(yRBF)

    # Display the plot
    plt.show()
