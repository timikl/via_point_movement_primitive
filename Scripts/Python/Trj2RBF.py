import numpy as np
import matplotlib.pyplot as plt
from RBF_Integrate import RBF_Integrate
from RBF_Train import RBF_Train

def Trj2RBF(y, dt, NN=30):
    """
    Realize trajectory q with RBFs.
    
    Parameters:
    - y: trajectory
    - dt: time step
    - NN: number of Gaussian kernel functions (default is 30)
    
    Returns:
    - RBF: RBF parameters
    - qRBF: RBF trajectory
    """
    
    joints = y.shape[1]
    
    # Learning
    RBF = {'N': NN, 'dt': dt, 'a_x': 2}
    RBF = RBF_Train(y, RBF)
    
    Y = []
    S = {'x': 1}
    
    # Due to rounding, add one more sample and hence the inequality
    while S['x'] >= np.exp(-RBF['a_x'] * (1 + RBF['dt'] / RBF['tau'])):
        S, arr = RBF_Integrate(RBF, S)
        Y.append(S['y'])
    
    Y = np.array(Y)
    
    # Plot if desired
    want_plot = False
    if want_plot:
        plt.figure(99)
        plt.plot(y, 'k')
        plt.plot(Y, 'r:')
        plt.title('trajectory')
        plt.show()
    
    qRBF = Y
    
    return RBF, qRBF


