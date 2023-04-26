import numpy as np

from validateargs import isvector, isscalar

def grad(fun, x0, dx=0.000001):
    x0 = np.asarray(x0)
    dx = np.asarray(dx)
    n = x0.size
    if n==1:
        if not x0.size==dx.size:
            raise ValueError('Parameters have to be same size')
        return (fun(x0+dx)-fun(x0-dx))/(2*dx)
    else:
        if isscalar(dx):
            dx = np.ones(x0.shape)*dx
        elif not x0.shape==dx.shape:
            raise ValueError('Parameters have to be same size')
        g = np.empty(n)
        for i in range(n):
            u = np.copy(x0)
            u[i] = x0[i]+dx[i]
            f1 = fun(u)
            u[i] = x0[i]-dx[i]
            f2 = fun(u)
            g[i] = (f1-f2)/(2*dx[i])
        return g

if __name__ == '__main__':

    fun = lambda x: np.sin(x)
    x = 0.
    print('Fun : ', fun(x))
    print('Grad: ', grad(fun, x))
