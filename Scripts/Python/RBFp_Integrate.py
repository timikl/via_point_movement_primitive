import numpy as np
def RBFp_Integrate(RBF, S, nu=1):
    """
    Discrete move RBF realization, NOT weighted with the phase.

    Parameters:
    - RBF: RBF parameters
    - S: current state
    - nu: optional parameter (default is 1)

    Returns:
    - S: updated state
    - psi: weighted sum of the locally weighted regression models
    """

    # Initialization
    epsilon = 1e-10
    NS = RBF['w'].shape[1]  # number of signals

    # The weighted sum of the locally weighted regression models
    psi = np.exp(-(S['x'] - RBF['c']) ** 2 / (2 * RBF['sigma2']))

    for i in range(NS):
        S['y'][i] = np.sum(RBF['w'][:, i] * psi)

    S['basis'] = psi * S['x']

    # Phase variable must be last updated
    dx = -RBF['a_x'] * S['x'] / RBF['tau'] * nu
    S['x'] = S['x'] + dx * RBF['dt']

    return S, psi