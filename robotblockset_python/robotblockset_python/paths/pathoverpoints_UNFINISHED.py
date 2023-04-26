import numpy as np
import uniqueCartesianPath


def pathoverpoints(pnt, **kwargs):
    """
    PATHOVERPOINTS Generates path over points using spline interpolation 
    %
    % Usage:
    %       [xi,si,xid,sid,xidd,sidd,xaux]=pathoverpoints(pnt)
    %       [...]=pathoverpoints(...,Options)
    %
    % Input:
    %       pnt     waypoints for path;
    %                   - Task points (positions) (npoints x 3)
    %                   - Task points (positions+quaternions) (npoints x 7) 
    %
    % Output:
    %       xi   path points  
    %       si   path parameter
    %       xid  path derivative 
    %       sid  path parameter derivative
    %       xidd  path 2nd derivative 
    %       sidd  path parameter 2nd derivative
    %       xaux auxilary points
    %
    % Options:
    %   Interp      Interpolation type: (default 'inner')
    %                   'inner'     spline by uniform subdivision
    %                   'spline'    cubic spline curve
    %                   'none'      no interpolation
    %   Step        maximal difference in path parameter (default 0.01)
    %   Npoints     minimal number of path points - if 0 then Step is used (default 0)
    %   Auxpoints   auxiliary points (default 'none'):
    %                   'absolute'  Auxdist parameter is absolute distance
    %                   'relative'  Auxdist parameter is relative distance of
    %                               path segment
    %                   'none'
    %   Auxdist     distance of auxiliary points (1 x 2)
    %   Order       order of inner spline (Default 4)
    %   Natural     make path parameter natural - 'on' or 'off' (default 'on')
    %   NormScale   Scaling factor for rotation norm
    %   Plot        flag for plotting - 'off' or 'on' (default 'off')
    %   Figure      Figure handle
    %
    %

    % Copyright (c) 2016 by IJS Leon Zlajpah
    %

    """
    # User-adjustable parameters default values
    defaults = {
        'Interp' : 'inner',
        'Step' : 0.01,
        'Npoints': 0,
        'Auxpoints': 'none',
        'Auxdist': [0.1, 0.1],
        'Order' : 4,
        'Natural' : 'off',
        'NormScale': 1,
        'Plot' :  'off',
        'Figure' : []}
    
    # Prepare inputs
    for key in defaults.keys():
        kwargs.setdefault(key, defaults[key])
        
    assert kwargs['Interp'] in ['inner', 'spline', 'none']
    assert kwargs['Auxpoints'] in ['absolute', 'relative', 'none']
    assert kwargs['Natural'] in ['on', 'off']
    assert kwargs['Plot'] in ['on', 'off']
    assert kwargs['Order'] >= 2
    
    if kwargs['Auxpoints'] == 'relative':
        assert 0 <= kwargs['Auxdist'] <= 0.5
    
    if kwargs['Natural'] == 'on':
        assert kwargs['NormScale'] >= 0
    
    if len(kwargs['Auxdist']) == 1:
        kwargs['Auxdist'] = [kwargs['Auxdist'], kwargs['Auxdist']]
        
    if pnt.ndim == 3:
        assert pnt.shape[0:1] == (4,4) # Maybe fix these indices
        xx = uniqueCartesianPath(t2x(pnt))
    else:
        nd = np.shape(pnt)[3] # Last axis size
        if nd not in [3,6,7]:
            raise Exception('Wrong input points dimension')
        if nd==6:
            xx = [pnt[:,0:3], rpy2q(pnt[:, 3:])]
        else:
            xx = np.transpose(pnt)
            
    points = np.transpose(xx)
    nd, np = np.shape(xx)
    
    # Add auxilliary points
    # TODO - copy this part of code
    
    if kwargs['Natural'] == 'on':
        kpoints = 4
    else:
        kpoints = 1
        
    # Inner spline interpolation
    if kwargs['interp'] == 'inner':
        ya = [xx[:,0] * np.ones(1, kwargs['Order']-2), xx, xx[:,-1]*np.ones(1, kwargs['Order'] - 2)]
        if kwargs['Npoints'] ==0:
            ys = spcrv(ya, kwargs['Order'])
            vx = diff(np.transpose(ys))
            sl = np.cumsum([0, np.dot(np.])

    
    
