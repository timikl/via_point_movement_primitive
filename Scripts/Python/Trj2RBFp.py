import numpy as np
from RBFp_Integrate import RBFp_Integrate
from RBFp_Train import RBFp_Train

def Trj2RBFp(y, dt, NN=30):
    if len(y.shape) == 1:
        y = y.reshape(-1, 1)

    joints = y.shape[1]
    RBF = {
        "N": NN,
        "dt": dt,
        "a_x": 2
    }

    RBF = RBFp_Train(y, RBF)  # Assuming RBFp_Train is another function you might provide later

    Y = []
    S = {"x": 1}

    while S["x"] >= np.exp(-RBF["a_x"] * (1 + RBF["dt"] / RBF["tau"])):
        S = RBFp_Integrate(RBF, S)  # Assuming RBFp_Integrate is another function you might provide later
        Y.append(S["y"])

    Y = np.array(Y)

    want_plot = False
    if want_plot:
        import matplotlib.pyplot as plt
        plt.figure(99)
        plt.plot(y, 'k')
        plt.plot(Y, 'r:')
        plt.title('trajectory')
        plt.show()

    qRBF = Y
    return RBF, qRBF
