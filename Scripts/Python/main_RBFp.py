import numpy as np
import matplotlib.pyplot as plt
from Trj2RBFp import Trj2RBFp
from RBFp2Trj import RBFp2Trj

if __name__ == "__main__":

    # Generating trajectory
    dt = 0.01
    tf = 2
    t = np.arange(0, tf + dt, dt)
    p = []

    # Plots 11 sinuses
    for a in np.arange(1, 2.1, 0.1):
        p.append(a * np.sin(t * 2 * np.pi / tf))

    #TODO: debug p size is not vector any more, it is a matrix, what to do?

    p = np.array(p)
    #p = p.reshape(-1, 1)
    print(p)

    plt.plot(t, p.T)
    plt.legend(["sin" + str(i) for i in range(1, 12)])

    # Mean and Standard deviation of sinuses
    mu = np.mean(p, axis=0)
    sigma = np.std(p, axis=0)

    plt.plot(t, mu, 'k', linewidth=2, label="mu")

    # RBF
    W = []
    M = 10
    for j in range(p.shape[0]):
        RBF, _ = Trj2RBFp(p[j, :], dt, M)  # Assuming Trj2RBFp is a defined function
        W.append(RBF['w'])

    W = np.array(W)
    mu_w = np.mean(W, axis=1)
    sigma_w = np.zeros((M, M))

    # Equation (5)
    for i in range(M):
        sigma_w += 1/M * np.outer(W[i, :] - mu_w, W[i, :] - mu_w)

    RBF_mu = RBF
    RBF_mu['w'] = mu_w

    p_w = RBFp2Trj(RBF_mu)  # Assuming RBFp2Trj is a defined function

    plt.plot(t[:len(p_w)], p_w, ':k', linewidth=3, label="RBF_mu")

    # Prediction mu_x based on via-point
    x_via = 1.5
    y_via = -1.9

    S = {'x': np.exp(-RBF['a_x'] * x_via / RBF['tau'])}
    sigma_y = 10.001

    # Equations (6) from Learning via Mov primitives article
    psi = np.exp(-(S['x'] - RBF_mu['c'])**2 / (2 * RBF_mu['sigma2']))
    L = sigma_w @ psi / (sigma_y + psi.T @ sigma_w @ psi) # @ is a binary operator used for matrix multiplication
    mu_v = mu_w + L * (y_via - psi.T @ mu_w)

    # Test
    RBF_via = RBF_mu
    RBF_via['w'] = mu_v

    p_via = RBFp2Trj(RBF_via)

    plt.plot(t[:len(p_via)], p_via, ':m', linewidth=2, label="RBF_via")

    # Set the legend
    labels = ["sin" + str(i) for i in range(1, 12)] + ['mu', 'RBF_mu', 'RBF_via']
    plt.legend(labels)

    plt.show()
