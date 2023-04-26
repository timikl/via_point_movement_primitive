"""Robot models

Copyright (c) 2018 by IJS Leon Zlajpah

"""
import numpy as np
from transformations import rp2t, rpy2r, q2r


def kinmodel_panda(q, tcp=None, out='x'):
    """ Generated model for FRANKA-Emika Panda

    Parameters
    ----------
    q : array of floats
        joint positions (nj,)
    tcp : array of floats
        tool center point

    Returns
    -------
    p : array of floats
        task position (3,)
    R : array of floats
        rotational matrix (3, 3)
    J : array of floats
        Jacobian matrix (6, nj)
    """

    c1 = np.cos(q[0])
    s1 = np.sin(q[0])
    c2 = np.cos(q[1])
    s2 = np.sin(q[1])
    c3 = np.cos(q[2])
    s3 = np.sin(q[2])
    c4 = np.cos(q[3])
    s4 = np.sin(q[3])
    c5 = np.cos(q[4])
    s5 = np.sin(q[4])
    c6 = np.cos(q[5])
    s6 = np.sin(q[5])
    c7 = np.cos(q[6])
    s7 = np.sin(q[6])

    a3 = 0.082500
    a4 = -0.082500
    a6 = 0.088000

    d1 = 0.333000
    d3 = 0.316000
    d5 = 0.384000
    d7 = 0.107000

    p = np.zeros(3)
    p[0] = d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + d3*c1*s2 - a3*s1*s3 + a6*s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - a6*c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) - a4*c4*(s1*s3 - c1*c2*c3) + a3*c1*c2*c3 + a4*c1*s2*s4
    p[1] = d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + a3*c1*s3 + d3*s1*s2 - a6*s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + a6*c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)) + a4*c4*(c1*s3 + c2*c3*s1) + a3*c2*c3*s1 + a4*s1*s2*s4
    p[2] = d1 + d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - c6*(c2*c4 + c3*s2*s4)) + d5*(c2*c4 + c3*s2*s4) + d3*c2 + a6*s6*(c2*c4 + c3*s2*s4) - a3*c3*s2 + a4*c2*s4 + a6*c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - a4*c3*c4*s2

    R = np.zeros((3,3))
    R[0, 0]=c7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) - s7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))
    R[0, 1]=- s7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) - c7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))
    R[0, 2]=- c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))
    R[1, 0]=s7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) - c7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))
    R[1, 1]=s7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) + c7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))
    R[1, 2]=c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))
    R[2, 0]=s7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + c7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + s6*(c2*c4 + c3*s2*s4))
    R[2, 1]=c7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - s7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + s6*(c2*c4 + c3*s2*s4))
    R[2, 2]=s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - c6*(c2*c4 + c3*s2*s4)

    Jp = np.zeros((3, 7))
    Jp[0, 0]=d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - a3*c1*s3 - d3*s1*s2 + a6*s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - a6*c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)) - a4*c4*(c1*s3 + c2*c3*s1) - a3*c2*c3*s1 - a4*s1*s2*s4
    Jp[0, 1]=d5*(c1*c2*c4 + c1*c3*s2*s4) - d7*(c6*(c1*c2*c4 + c1*c3*s2*s4) - s6*(c5*(c1*c2*s4 - c1*c3*c4*s2) + c1*s2*s3*s5)) + a6*c6*(c5*(c1*c2*s4 - c1*c3*c4*s2) + c1*s2*s3*s5) + d3*c1*c2 + a6*s6*(c1*c2*c4 + c1*c3*s2*s4) - a3*c1*c3*s2 + a4*c1*c2*s4 - a4*c1*c3*c4*s2
    Jp[0, 2]=d7*(s6*(s5*(s1*s3 - c1*c2*c3) - c4*c5*(c3*s1 + c1*c2*s3)) - c6*s4*(c3*s1 + c1*c2*s3)) + a6*c6*(s5*(s1*s3 - c1*c2*c3) - c4*c5*(c3*s1 + c1*c2*s3)) + d5*s4*(c3*s1 + c1*c2*s3) - a3*c3*s1 - a4*c4*(c3*s1 + c1*c2*s3) - a3*c1*c2*s3 + a6*s4*s6*(c3*s1 + c1*c2*s3)
    Jp[0, 3]=d5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - d7*(c6*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)) + a4*s4*(s1*s3 - c1*c2*c3) + a6*s6*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + a4*c1*c4*s2 + a6*c5*c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)
    Jp[0, 4]=a6*c6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + d7*s6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))
    Jp[0, 5]=d7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + a6*c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + a6*s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))
    Jp[0, 6]=0
    Jp[1, 0]=d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + d3*c1*s2 - a3*s1*s3 + a6*s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - a6*c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) - a4*c4*(s1*s3 - c1*c2*c3) + a3*c1*c2*c3 + a4*c1*s2*s4
    Jp[1, 1]=d5*(c2*c4*s1 + c3*s1*s2*s4) - d7*(c6*(c2*c4*s1 + c3*s1*s2*s4) - s6*(c5*(c2*s1*s4 - c3*c4*s1*s2) + s1*s2*s3*s5)) + a6*c6*(c5*(c2*s1*s4 - c3*c4*s1*s2) + s1*s2*s3*s5) + d3*c2*s1 + a6*s6*(c2*c4*s1 + c3*s1*s2*s4) - a3*c3*s1*s2 + a4*c2*s1*s4 - a4*c3*c4*s1*s2
    Jp[1, 2]=a3*c1*c3 - a6*c6*(s5*(c1*s3 + c2*c3*s1) - c4*c5*(c1*c3 - c2*s1*s3)) - d5*s4*(c1*c3 - c2*s1*s3) - d7*(s6*(s5*(c1*s3 + c2*c3*s1) - c4*c5*(c1*c3 - c2*s1*s3)) - c6*s4*(c1*c3 - c2*s1*s3)) + a4*c4*(c1*c3 - c2*s1*s3) - a3*c2*s1*s3 - a6*s4*s6*(c1*c3 - c2*s1*s3)
    Jp[1, 3]=d7*(c6*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)) - d5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - a4*s4*(c1*s3 + c2*c3*s1) - a6*s6*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + a4*c4*s1*s2 - a6*c5*c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)
    Jp[1, 4]=- a6*c6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) - d7*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))
    Jp[1, 5]=- d7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - a6*c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - a6*s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))
    Jp[1, 6]=0
    Jp[2, 0]=0
    Jp[2, 1]=- d5*(c4*s2 - c2*c3*s4) - d3*s2 - d7*(s6*(c5*(s2*s4 + c2*c3*c4) - c2*s3*s5) - c6*(c4*s2 - c2*c3*s4)) - a6*s6*(c4*s2 - c2*c3*s4) - a3*c2*c3 - a4*s2*s4 - a6*c6*(c5*(s2*s4 + c2*c3*c4) - c2*s3*s5) - a4*c2*c3*c4
    Jp[2, 2]=d7*(s6*(c3*s2*s5 + c4*c5*s2*s3) + c6*s2*s3*s4) + a3*s2*s3 + a6*c6*(c3*s2*s5 + c4*c5*s2*s3) + a4*c4*s2*s3 - d5*s2*s3*s4 - a6*s2*s3*s4*s6
    Jp[2, 3]=d7*(c6*(c2*s4 - c3*c4*s2) + c5*s6*(c2*c4 + c3*s2*s4)) - d5*(c2*s4 - c3*c4*s2) - a6*s6*(c2*s4 - c3*c4*s2) + a4*c2*c4 + a4*c3*s2*s4 + a6*c5*c6*(c2*c4 + c3*s2*s4)
    Jp[2, 4]=- a6*c6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - d7*s6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)
    Jp[2, 5]=d7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + s6*(c2*c4 + c3*s2*s4)) - a6*s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + a6*c6*(c2*c4 + c3*s2*s4)
    Jp[2, 6]=0

    Jr=np.zeros((3, 7))
    Jr[0, 0]=0
    Jr[0, 1]=-s1
    Jr[0, 2]=c1*s2
    Jr[0, 3]=c3*s1 + c1*c2*s3
    Jr[0, 4]=s4*(s1*s3 - c1*c2*c3) + c1*c4*s2
    Jr[0, 5]=c5*(c3*s1 + c1*c2*s3) - s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4)
    Jr[0, 6]=- c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))
    Jr[1, 0]=0
    Jr[1, 1]=c1
    Jr[1, 2]=s1*s2
    Jr[1, 3]=c2*s1*s3 - c1*c3
    Jr[1, 4]=c4*s1*s2 - s4*(c1*s3 + c2*c3*s1)
    Jr[1, 5]=s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)
    Jr[1, 6]=c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))
    Jr[2, 0]=1
    Jr[2, 1]=0
    Jr[2, 2]=c2
    Jr[2, 3]=-s2*s3
    Jr[2, 4]=c2*c4 + c3*s2*s4
    Jr[2, 5]=s5*(c2*s4 - c3*c4*s2) - c5*s2*s3
    Jr[2, 6]=s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - c6*(c2*c4 + c3*s2*s4)

    if tcp is not None:
        if tcp.shape == (4, 4):
            p_tcp = tcp[:3, 3]
            R_tcp = tcp[:3, :3]
        elif len(tcp) == 7:
            p_tcp = tcp[:3]
            R_tcp = q2r(tcp[3:7])
        elif len(tcp) == 6:
            p_tcp = tcp[:3]
            R_tcp = rpy2r(tcp[3:6], out='R')
        else:
            raise ValueError('kinmodel: tcp is not SE3')
        v = R@p_tcp
        s = np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]])
        p = p+R@p_tcp
        Jp = Jp+s.T@Jr
        R = R@R_tcp

    J = np.vstack((Jp,Jr))

    if out=='pR':
        return p, R, J
    else:
        return rp2t(R=R, p=p, out=out), J


def kinmodel_lwr(q, tcp=None, out='x'):
    """ Generated model for KUKA LWR

    Parameters
    ----------
    q : array of floats
        joint positions (nj,)
    tcp : array of floats
        tool center point

    Returns
    -------
    p : array of floats
        task position (3,)
    R : array of floats
        rotational matrix (3, 3)
    J : array of floats
        Jacobian matrix (6, nj)
    """

    c1 = np.cos(q[0])
    s1 = np.sin(q[0])
    c2 = np.cos(q[1])
    s2 = np.sin(q[1])
    c3 = np.cos(q[2])
    s3 = np.sin(q[2])
    c4 = np.cos(q[3])
    s4 = np.sin(q[3])
    c5 = np.cos(q[4])
    s5 = np.sin(q[4])
    c6 = np.cos(q[5])
    s6 = np.sin(q[5])
    c7 = np.cos(q[6])
    s7 = np.sin(q[6])


    d1 = 0.310000
    d3 = 0.400000
    d5 = 0.390000
    d7 = 0.078000

    p = np.zeros(3)
    p[0] = - d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) - d3*c1*s2
    p[1] = d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - d3*s1*s2
    p[2] = d1 + d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + d5*(c2*c4 + c3*s2*s4) + d3*c2

    R = np.zeros((3,3))
    R[0, 0]=s7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - c7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))
    R[0, 1]=s7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + c7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))
    R[0, 2]=s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) - c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)
    R[1, 0]=c7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - s7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))
    R[1, 1]=- s7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - c7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))
    R[1, 2]=c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))
    R[2, 0]=s7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - c7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4))
    R[2, 1]=c7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + s7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4))
    R[2, 2]=s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)

    Jp = np.zeros((3, 7))
    Jp[0, 0]=d3*s1*s2 - d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)
    Jp[0, 1]=- d7*(c6*(c1*c2*c4 + c1*c3*s2*s4) + s6*(c5*(c1*c2*s4 - c1*c3*c4*s2) + c1*s2*s3*s5)) - d5*(c1*c2*c4 + c1*c3*s2*s4) - d3*c1*c2
    Jp[0, 2]=- d7*(s6*(s5*(s1*s3 - c1*c2*c3) - c4*c5*(c3*s1 + c1*c2*s3)) + c6*s4*(c3*s1 + c1*c2*s3)) - d5*s4*(c3*s1 + c1*c2*s3)
    Jp[0, 3]=- d7*(c6*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + c5*s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)) - d5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4)
    Jp[0, 4]=-d7*s6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))
    Jp[0, 5]=d7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))
    Jp[0, 6]=0
    Jp[1, 0]=- d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) - d3*c1*s2
    Jp[1, 1]=- d7*(c6*(c2*c4*s1 + c3*s1*s2*s4) + s6*(c5*(c2*s1*s4 - c3*c4*s1*s2) + s1*s2*s3*s5)) - d5*(c2*c4*s1 + c3*s1*s2*s4) - d3*c2*s1
    Jp[1, 2]=d7*(s6*(s5*(c1*s3 + c2*c3*s1) - c4*c5*(c1*c3 - c2*s1*s3)) + c6*s4*(c1*c3 - c2*s1*s3)) + d5*s4*(c1*c3 - c2*s1*s3)
    Jp[1, 3]=d7*(c6*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + c5*s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)) + d5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4)
    Jp[1, 4]=d7*s6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))
    Jp[1, 5]=-d7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))
    Jp[1, 6]=0
    Jp[2, 0]=0
    Jp[2, 1]=- d5*(c4*s2 - c2*c3*s4) - d3*s2 - d7*(s6*(c5*(s2*s4 + c2*c3*c4) - c2*s3*s5) + c6*(c4*s2 - c2*c3*s4))
    Jp[2, 2]=d7*(s6*(c3*s2*s5 + c4*c5*s2*s3) - c6*s2*s3*s4) - d5*s2*s3*s4
    Jp[2, 3]=- d5*(c2*s4 - c3*c4*s2) - d7*(c6*(c2*s4 - c3*c4*s2) - c5*s6*(c2*c4 + c3*s2*s4))
    Jp[2, 4]=-d7*s6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)
    Jp[2, 5]=d7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4))
    Jp[2, 6]=0

    Jr=np.zeros((3, 7))
    Jr[0, 0]=0
    Jr[0, 1]=s1
    Jr[0, 2]=-c1*s2
    Jr[0, 3]=- c3*s1 - c1*c2*s3
    Jr[0, 4]=- s4*(s1*s3 - c1*c2*c3) - c1*c4*s2
    Jr[0, 5]=c5*(c3*s1 + c1*c2*s3) - s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4)
    Jr[0, 6]=s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) - c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)
    Jr[1, 0]=0
    Jr[1, 1]=-c1
    Jr[1, 2]=-s1*s2
    Jr[1, 3]=c1*c3 - c2*s1*s3
    Jr[1, 4]=s4*(c1*s3 + c2*c3*s1) - c4*s1*s2
    Jr[1, 5]=s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)
    Jr[1, 6]=c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))
    Jr[2, 0]=1
    Jr[2, 1]=0
    Jr[2, 2]=c2
    Jr[2, 3]=-s2*s3
    Jr[2, 4]=c2*c4 + c3*s2*s4
    Jr[2, 5]=c5*s2*s3 - s5*(c2*s4 - c3*c4*s2)
    Jr[2, 6]=s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)

    if tcp is not None:
        if tcp.shape == (4, 4):
            p_tcp = tcp[:3, 3]
            R_tcp = tcp[:3, :3]
        elif len(tcp) == 7:
            p_tcp = tcp[:3]
            R_tcp = q2r(tcp[3:7])
        elif len(tcp) == 6:
            p_tcp = tcp[:3]
            R_tcp = rpy2r(tcp[3:6], out='R')
        else:
            raise ValueError('kinmodel: tcp is not SE3')
        v = R@p_tcp
        s = np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]])
        p = p+R@p_tcp
        Jp = Jp+s.T@Jr
        R = R@R_tcp

    J = np.vstack((Jp,Jr))

    if out=='pR':
        return p, R, J
    else:
        return rp2t(R=R, p=p, out=out), J


def kinmodel_ur5(q, tcp=None, out='x'):
    """ Generated model for UR5

    Parameters
    ----------
    q : array of floats
        joint positions (nj,)
    tcp : array of floats
        tool center point

    Returns
    -------
    p : array of floats
        task position (3,)
    R : array of floats
        rotational matrix (3, 3)
    J : array of floats
        Jacobian matrix (6, nj)
    """

    c1 = np.cos(q[0])
    s1 = np.sin(q[0])
    c2 = np.cos(q[1])
    s2 = np.sin(q[1])
    c3 = np.cos(q[2])
    s3 = np.sin(q[2])
    c4 = np.cos(q[3])
    s4 = np.sin(q[3])
    c5 = np.cos(q[4])
    s5 = np.sin(q[4])
    c6 = np.cos(q[5])
    s6 = np.sin(q[5])

    a2 = -0.425000
    a3 = -0.392250

    d1 = 0.109150
    d4 = 0.109150
    d5 = 0.094560
    d6 = 0.082300

    p = np.zeros(3)
    p[0] = d5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)) + d4*s1 + d6*(c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))) + a2*c1*c2 + a3*c1*c2*c3 - a3*c1*s2*s3
    p[1] = d5*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)) - d4*c1 - d6*(c1*c5 - s5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))) + a2*c2*s1 + a3*c2*c3*s1 - a3*s1*s2*s3
    p[2] = d1 + a2*s2 - d5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2)) + a3*c2*s3 + a3*c3*s2 - d6*s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3))

    R = np.zeros((3,3))
    R[0, 0]=c6*(s1*s5 + c5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))) - s6*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3))
    R[0, 1]=- s6*(s1*s5 + c5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))) - c6*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3))
    R[0, 2]=c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))
    R[1, 0]=- c6*(c1*s5 + c5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))) - s6*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1))
    R[1, 1]=s6*(c1*s5 + c5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))) - c6*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1))
    R[1, 2]=s5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2)) - c1*c5
    R[2, 0]=s6*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2)) + c5*c6*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3))
    R[2, 1]=c6*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2)) - c5*s6*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3))
    R[2, 2]=-s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3))

    Jp = np.zeros((3, 6))
    Jp[0, 0]=d4*c1 - d5*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)) + d6*(c1*c5 - s5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))) - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3
    Jp[0, 1]=d5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)) - a2*c1*s2 + d6*s5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)) - a3*c1*c2*s3 - a3*c1*c3*s2
    Jp[0, 2]=d5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)) + d6*s5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)) - a3*c1*c2*s3 - a3*c1*c3*s2
    Jp[0, 3]=d5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)) + d6*s5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3))
    Jp[0, 4]=-d6*(s1*s5 + c5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)))
    Jp[0, 5]=0
    Jp[1, 0]=d5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)) + d4*s1 + d6*(c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))) + a2*c1*c2 + a3*c1*c2*c3 - a3*c1*s2*s3
    Jp[1, 1]=d6*s5*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)) - d5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2)) - a2*s1*s2 - a3*c2*s1*s3 - a3*c3*s1*s2
    Jp[1, 2]=d6*s5*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)) - d5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2)) - a3*c2*s1*s3 - a3*c3*s1*s2
    Jp[1, 3]=d6*s5*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)) - d5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))
    Jp[1, 4]=d6*(c1*s5 + c5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2)))
    Jp[1, 5]=0
    Jp[2, 0]=0
    Jp[2, 1]=a2*c2 + d5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)) + a3*c2*c3 - a3*s2*s3 - d6*s5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2))
    Jp[2, 2]=d5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)) + a3*c2*c3 - a3*s2*s3 - d6*s5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2))
    Jp[2, 3]=d5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)) - d6*s5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2))
    Jp[2, 4]=-d6*c5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3))
    Jp[2, 5]=0

    Jr=np.zeros((3, 6))
    Jr[0, 0]=0
    Jr[0, 1]=s1
    Jr[0, 2]=s1
    Jr[0, 3]=s1
    Jr[0, 4]=c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)
    Jr[0, 5]=c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))
    Jr[1, 0]=0
    Jr[1, 1]=-c1
    Jr[1, 2]=-c1
    Jr[1, 3]=-c1
    Jr[1, 4]=c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)
    Jr[1, 5]=s5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2)) - c1*c5
    Jr[2, 0]=1
    Jr[2, 1]=0
    Jr[2, 2]=0
    Jr[2, 3]=0
    Jr[2, 4]=s4*(c2*s3 + c3*s2) - c4*(c2*c3 - s2*s3)
    Jr[2, 5]=-s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3))

    if tcp is not None:
        if tcp.shape == (4, 4):
            p_tcp = tcp[:3, 3]
            R_tcp = tcp[:3, :3]
        elif len(tcp) == 7:
            p_tcp = tcp[:3]
            R_tcp = q2r(tcp[3:7])
        elif len(tcp) == 6:
            p_tcp = tcp[:3]
            R_tcp = rpy2r(tcp[3:6], out='R')
        else:
            raise ValueError('kinmodel: tcp is not SE3')
        v = R@p_tcp
        s = np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]])
        p = p+R@p_tcp
        Jp = Jp+s.T@Jr
        R = R@R_tcp

    J = np.vstack((Jp,Jr))

    if out=='pR':
        return p, R, J
    else:
        return rp2t(R=R, p=p, out=out), J


def kinmodel_ur10(q, tcp=None, out='x'):
    """ Generated model for UR10

    Parameters
    ----------
    q : array of floats
        joint positions (nj,)
    tcp : array of floats
        tool center point

    Returns
    -------
    p : array of floats
        task position (3,)
    R : array of floats
        rotational matrix (3, 3)
    J : array of floats
        Jacobian matrix (6, nj)
    """

    c1 = np.cos(q[0])
    s1 = np.sin(q[0])
    c2 = np.cos(q[1])
    s2 = np.sin(q[1])
    c3 = np.cos(q[2])
    s3 = np.sin(q[2])
    c4 = np.cos(q[3])
    s4 = np.sin(q[3])
    c5 = np.cos(q[4])
    s5 = np.sin(q[4])
    c6 = np.cos(q[5])
    s6 = np.sin(q[5])

    a2 = -0.612000
    a3 = -0.572300

    d1 = 0.127300
    d4 = 0.163941
    d5 = 0.115700
    d6 = 0.092200

    p = np.zeros(3)
    p[0] = d5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)) + d4*s1 + d6*(c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))) + a2*c1*c2 + a3*c1*c2*c3 - a3*c1*s2*s3
    p[1] = d5*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)) - d4*c1 - d6*(c1*c5 - s5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))) + a2*c2*s1 + a3*c2*c3*s1 - a3*s1*s2*s3
    p[2] = d1 + a2*s2 - d5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2)) + a3*c2*s3 + a3*c3*s2 - d6*s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3))

    R = np.zeros((3,3))
    R[0, 0]=c6*(s1*s5 + c5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))) - s6*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3))
    R[0, 1]=- s6*(s1*s5 + c5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))) - c6*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3))
    R[0, 2]=c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))
    R[1, 0]=- c6*(c1*s5 + c5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))) - s6*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1))
    R[1, 1]=s6*(c1*s5 + c5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))) - c6*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1))
    R[1, 2]=s5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2)) - c1*c5
    R[2, 0]=s6*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2)) + c5*c6*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3))
    R[2, 1]=c6*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2)) - c5*s6*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3))
    R[2, 2]=-s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3))

    Jp = np.zeros((3, 6))
    Jp[0, 0]=d4*c1 - d5*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)) + d6*(c1*c5 - s5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))) - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3
    Jp[0, 1]=d5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)) - a2*c1*s2 + d6*s5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)) - a3*c1*c2*s3 - a3*c1*c3*s2
    Jp[0, 2]=d5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)) + d6*s5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)) - a3*c1*c2*s3 - a3*c1*c3*s2
    Jp[0, 3]=d5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)) + d6*s5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3))
    Jp[0, 4]=-d6*(s1*s5 + c5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)))
    Jp[0, 5]=0
    Jp[1, 0]=d5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)) + d4*s1 + d6*(c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))) + a2*c1*c2 + a3*c1*c2*c3 - a3*c1*s2*s3
    Jp[1, 1]=d6*s5*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)) - d5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2)) - a2*s1*s2 - a3*c2*s1*s3 - a3*c3*s1*s2
    Jp[1, 2]=d6*s5*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)) - d5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2)) - a3*c2*s1*s3 - a3*c3*s1*s2
    Jp[1, 3]=d6*s5*(c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)) - d5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2))
    Jp[1, 4]=d6*(c1*s5 + c5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2)))
    Jp[1, 5]=0
    Jp[2, 0]=0
    Jp[2, 1]=a2*c2 + d5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)) + a3*c2*c3 - a3*s2*s3 - d6*s5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2))
    Jp[2, 2]=d5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)) + a3*c2*c3 - a3*s2*s3 - d6*s5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2))
    Jp[2, 3]=d5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)) - d6*s5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2))
    Jp[2, 4]=-d6*c5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3))
    Jp[2, 5]=0

    Jr=np.zeros((3, 6))
    Jr[0, 0]=0
    Jr[0, 1]=s1
    Jr[0, 2]=s1
    Jr[0, 3]=s1
    Jr[0, 4]=c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)
    Jr[0, 5]=c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2))
    Jr[1, 0]=0
    Jr[1, 1]=-c1
    Jr[1, 2]=-c1
    Jr[1, 3]=-c1
    Jr[1, 4]=c4*(c2*s1*s3 + c3*s1*s2) - s4*(s1*s2*s3 - c2*c3*s1)
    Jr[1, 5]=s5*(c4*(s1*s2*s3 - c2*c3*s1) + s4*(c2*s1*s3 + c3*s1*s2)) - c1*c5
    Jr[2, 0]=1
    Jr[2, 1]=0
    Jr[2, 2]=0
    Jr[2, 3]=0
    Jr[2, 4]=s4*(c2*s3 + c3*s2) - c4*(c2*c3 - s2*s3)
    Jr[2, 5]=-s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3))

    if tcp is not None:
        if tcp.shape == (4, 4):
            p_tcp = tcp[:3, 3]
            R_tcp = tcp[:3, :3]
        elif len(tcp) == 7:
            p_tcp = tcp[:3]
            R_tcp = q2r(tcp[3:7])
        elif len(tcp) == 6:
            p_tcp = tcp[:3]
            R_tcp = rpy2r(tcp[3:6], out='R')
        else:
            raise ValueError('kinmodel: tcp is not SE3')
        v = R@p_tcp
        s = np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]])
        p = p+R@p_tcp
        Jp = Jp+s.T@Jr
        R = R@R_tcp

    J = np.vstack((Jp,Jr))

    if out=='pR':
        return p, R, J
    else:
        return rp2t(R=R, p=p, out=out), J


if __name__ == '__main__':
    np.set_printoptions(formatter={'float': '{: 0.4f}'.format})
    q = np.array([0, -0.2, 0, -1.5, 0, 1.5, 0.7854])
    tcp = np.array([[0.7071, 0.7071, 0, 0], [-0.7071, 0.7071, 0, 0], [0, 0, 1, 0.1034], [0, 0, 0, 1]])
    p, R, J = kinmodel_panda(q, out='pR')
    print('p:', p )
    print('R')
    print(R)
