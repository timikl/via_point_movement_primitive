import numpy as np


def RBFp_Train(y, RBF, nu=None):
    """
    Global regression nonormalized RBF (PMP).

    Parameters:
    - y: signal
    - RBF: RBF parameters
    - nu: optional parameter

    RBF parameters:
    - N: number of Gaussian kernel functions
    - w: weight vector of size(Nx1)
    - c, sigma2, tau
    """

    # Check if nu is provided
    if nu is None:
        nu = np.ones(y.shape[0])

    # params - global
    NT, NS = y.shape
    RBF['tau'] = (NT - 1) * RBF['dt']
    RBF['w'] = np.zeros((RBF['N'], NS))  # initial weights

    # Gaussian kernel functions
    c_lin = np.linspace(0, 1, RBF['N'])
    RBF['c'] = np.exp(-RBF['a_x'] * c_lin)
    RBF['sigma2'] = np.diff(RBF['c']) * 0.75
    RBF['sigma2'] = np.append(RBF['sigma2'], RBF['sigma2'][-1]) ** 2

    # init params for target traj. and fitting
    x = 1
    A = np.zeros((NT, RBF['N']))

    # fit all points of the trajectory
    for t in range(NT):
        # the weighted sum of the locally weighted regression models
        psi = np.exp(-0.5 * (x - RBF['c']) ** 2 / RBF['sigma2'])
        A[t, :] = psi

        # update phase at last !!!!
        dx = -RBF['a_x'] * x / RBF['tau'] * nu[t]
        x = x + dx * RBF['dt']

    AI = np.linalg.pinv(A)
    RBF['w'] = np.dot(AI, y)

    return RBF