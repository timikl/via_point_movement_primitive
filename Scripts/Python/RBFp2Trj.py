import numpy as np
from RBFp_Integrate import RBFp_Integrate
def RBFp2Trj(RBF):
    y = []
    S = {"x": 1}

    while S["x"] >= np.exp(-RBF["a_x"]):
        S = RBFp_Integrate(RBF, S)  # Assuming RBFp_Integrate is another function you might provide later
        y.append(S["y"])

    y = np.array(y)
    return y