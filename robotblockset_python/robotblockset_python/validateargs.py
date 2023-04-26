import numpy as np

# valid scalar types
_scalartypes = (int, float, np.int64, np.int32, np.float64)
_eps = np.finfo(np.float64).eps

def isscalar(x) -> bool:
    """Check if parameter is scalar number

    Parameters
    ----------
    x : any
        value to check

    Returns
    -------
    bool
        True if x is int or real  
    """
    return isinstance(x, _scalartypes) or (isinstance(x, np.ndarray) and (x.size==1))

def isvector(x, dim=None) -> bool:
    """Check if parameter is a vector 

    Parameters
    ----------
    x : any
        value to check
    dim : int, optional
        required dimension; default 3

    Returns
    -------
    bool
        True if x has required dimension   
    """

    x = np.asarray(x)
    s = x.shape
    if dim is None:
        return (len(s)==1 and s[0]>1) or \
               (len(s)==2 and (s[0]==1 and s[1]>1) or (s[0]>1 and s[1]==1))
    else:
        return s==(dim,) or s==(1, dim) or s==(dim, 1)

def vector(x, dim=None):
    """Return a vector

    Parameters
    ----------
    x : array-like
        values to be transformed to vector
    dim : int
        required dimension; None: no length check

    Returns
    -------
    ndarray
        array in specified format
    """
    if isinstance(x, (list, tuple)):
        x = np.asarray(x)
    elif isscalar(x):
        x = np.asarray(x)
    elif isinstance(x, np.ndarray):
        x = x.flatten()
    else:
        raise TypeError("Invalid input type")
    if (dim is not None) and x.size != dim:
        raise ValueError("Incorrect vector length")
    return x

def ismatrix(x, shape=None) -> bool:
    """Check if parameter is a matrix

    Tests if the argument is a 2D matrix with a specified ``shape``.
    If ``shape`` is scalar, then only the  last dimension of the argument
    is checked.

    Parameters
    ----------
    x : ndarray
        value to check
    shape : tuple or scalar, optional
        required 2D shape

    Returns
    -------
    bool
        True if x has required dimensions   
    """
    if isinstance(x, np.ndarray):
        if shape is None:
            return len(x.shape)==2
        elif isscalar(shape) or len(shape)==1:
            xshape = x.shape
            return xshape[-1]==shape
        else:
            return x.shape==shape
    else:
        return False

def matrix(x, shape=None):
    """Return a matrix

    Parameters
    ----------
    x : array-like
        values to be transformed to matrix
    shape : tuple, optional
        required 2D shape
    
    Returns
    -------
    ndarray
        2D ndarray of specified shape
    """
    if isscalar(x) or isinstance(x, (list, tuple)):
        x = np.asarray(x)
    if not isinstance(x, np.ndarray):
        raise TypeError('Invalid argument type')
    if shape is None:
        if not x.ndim==2:
            raise TypeError('Argument is not two-dimensional array')
    else:
        if x.shape==shape:
            return x
        elif np.prod(x.shape)==np.prod(shape):
            return x.reshape(shape)
        else:
            raise ValueError(f"Cannot reshape {x.shape} to {shape}")

def check_shape(x, shape) -> bool:
    """Check last dimensions of np array

    Parameters
    ----------
    x : array-like
        array to be checked
    shape : tuple
        required dimension
    
    Returns
    -------
    bool
        True if parameters is (..., shape)
    """
    if shape==1:
        return isscalar(x)
    elif isinstance(x, (list, tuple)):
        x = np.asarray(x)
    elif isinstance(x, np.ndarray):
        pass
    else:
        raise TypeError("Invalid input type")
    if isscalar(shape):
        return x.shape[-1] == shape
    else:
        return x.shape[-len(shape):] == shape

def isskewsymmetric(S, tol=100) -> bool:
    """Check if matrix is skew-symmetric

    Parameters
    ----------
    S : ndarray
        value to check

    Returns
    -------
    bool
        True if S is skew-symmetric
    """
    return isinstance(S, np.ndarray) and np.linalg.norm(S+S.T)<tol*_eps

def getunit(unit: str) -> float:
    """Calculates unit conversion factor

    Parameters
    ----------
    unit : str, optional
        angular unit, by default 'rad'

    Returns
    -------
    float
        unit -> rad conversion factor 

    Raises
    ------
    ValueError
        Invalid unit
    """
    if unit.lower()=='rad':
        return 1
    elif unit.lower()=='deg':
        return np.pi/180
    else:
        raise ValueError("Invalid units")

def check_option(opt: str, val: str) -> bool:
    """Check is option equals value (case-independent)

    Parameters
    ----------
    opt : str
        option to be checked
    val : str
        value for check

    Returns
    -------
    bool
        Check result

    Note
    ----
    For check the shortest string length is used
    """
    siz = min(len(opt), len(val))
    return opt[:siz].lower()==val[:siz].lower()


if __name__ == '__main__':
    a = vector((1, 2,3), dim=3)
    print(a)

    b = matrix((1, 2, 3, 4), shape=(2, 2))
    print(b)

    x = np.empty((7,3,3))
    print('Check shape: ', check_shape(x, (3, 3)))

    print('Check shape: ', check_shape(a, 3))

