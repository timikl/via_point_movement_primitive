#!/usr/bin/env python3
import numpy as np


def polynomial1(x0, dx0, ddx0, x1, dx1, ddx1, T):
    '''
    Calculates the coefficients of minimum jerk polynomial

    x0, dx0, ddx0: initial position, velocity, and acceleration
    x1, dx1, ddx1: final position, velocity, and acceleration
    T: Time to move from x0 to x1

    y = a5 * t**5 + a4 * t**4 + a3 * t**3 + a2 * t**2 + a1 * t + a0
    '''

    a0 = x0
    a1 = dx0
    a2 = ddx0 / 2
    a3 = (20 * x1 - 20 * x0 - (8 * dx1 + 12 * dx0) * T - (3 * ddx0 - ddx1) * T**2 ) / (2 * T**3)
    a4 = (30 * x0 - 30 * x1 + (14 * dx1 + 16 * dx0) * T + (3 * ddx0 - 2 * ddx1) * T**2 ) / (2 * T**4)
    a5 = (12 * x1 - 12 * x0 - (6 * dx1 + 6 * dx0) * T - (ddx0 - ddx1) * T**2 ) / (2 * T**5)
    a543210 = [a5, a4, a3, a2, a1, a0]

    return a543210


def polynomial2(a543210, dt, T):
    '''
    Calculates positions, velocities and accelerations of minimum jerk
    polynomial from 0 to T with time step dt

    y = a5 * t**5 + a4 * t**4 + a3 * t**3 + a2 * t**2 + a1 * t + a0
    T must be the same as in polynomial1(...)
    '''

    y = np.zeros(int(T/dt + 1))  # must be divisible
    dy = np.zeros(int(T/dt + 1))  # must be divisible
    ddy = np.zeros(int(T/dt + 1))  # must be divisible

    for i in range(len(y)):
        i_t = i*dt
        y[i] = a543210[0] * i_t**5 + a543210[1] * i_t**4 + a543210[2] * i_t**3 + a543210[3] * i_t**2 + a543210[4] * i_t + a543210[5]
        dy[i] = 5 * a543210[0] * i_t**4 + 4 * a543210[1] * i_t**3 + 3 * a543210[2] * i_t**2 + 2 * a543210[3] * i_t + a543210[4]
        ddy[i] = 20 * a543210[0] * i_t**3 + 12 * a543210[1] * i_t**2 + 6 * a543210[2] * i_t + 2 * a543210[3]

    return (y, dy, ddy)
