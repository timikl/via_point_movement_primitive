# DMP Library
#
# Authors: Timotej Gaspar, Miha Denisa, Rok Pahic
# E-mail: timotej.gaspar@ijs.si

# Importing the module 'quaternion' will throw a warning if we do not have numba.
# I therefore set the warnings filter to 'ignore'.
import warnings
warnings.filterwarnings("ignore")

# Numpy
import numpy as np  # nopep8 - Oterwise the linter complainst for E402
import quaternion  # nopep8 - Oterwise the linter complainst for E402


class DMP(object):
    """DMP Class.

    This is the class that implements the Dynamic Movement Primitives - DMP.
    The class supports both Encoding and Decoding of trajectories.

    Attributes
    ----------
    num_weights : int
        Number of DMP weights.
    a_x : float
        The a_x gain of the DMP.
    a_z : float
        The a_z gain of the DMP.
    b_z : float
        The b_z gain of the DMP.
    c : array like
        The width of the Gaussian bell curves.
    y0 : 1 dimensional array
        The initial position of the DMP
    goal : 1 dimensional array
        The final position of the DMP
    sigma : array of floats
        The values of the Gaussian bell curves.
    num_dof : int
        The number of weights used for estimating the DMP.
    weights_pos : array of floats
        The calulated weights of the DMP.

    Parameters
    ----------
    pos_data : np.array or float list
        Data points of the trajectory to encode.
    vel_data : np.array or float list (optional)
        Data points of the trajectory's velocity to encode.
    time : np.array or float list or float
        If the provided time is an array of floats with the same length as the
        `pos_data` parameter, it will be treated as the time samlpes of the trajecoty.
        In case this parameter is a single float, it will be treated as the sample
        time of the trajectory and the time vector will be computed accordingly.
    num_weights : int, optional
        The desired number of weights to estimate the DMP.
    a_z : float, optional
        The a_z gain of the DMP.
    a_x : float, optional
        the a_x gain of the DMP.

    """
    a_x = None
    a_z = None
    b_z = None
    num_weights = None
    y0 = None
    goal = None
    _num_dof = None
    _d_t = None

    def __init__(self, pos_data=None, time=None, vel_data=[], num_weights=25, a_z=48.0, a_x=2.0):

        if pos_data is not None:
            # Copy the parameters into the appropriate class attributes
            self.a_x = a_x
            self.a_z = a_z
            self.b_z = self.a_z / 4
            self.num_weights = num_weights
            self._pos_training_data = np.asarray(pos_data)
            self._vel_training_data = np.asarray(vel_data)
            self.y0 = self._pos_training_data[0, :]
            self.goal = self._pos_training_data[-1, :]
            # We assume that the number of trajectory samples are smaller then the DOF
            self._num_samples = np.max(self._pos_training_data.shape)
            self._num_dof = np.min(self._pos_training_data.shape)

            try:
                if not(len(np.asarray(time).shape)):
                    # If the provided argument is a number we treat is as sample time
                    self._time_vec = np.arange(0, self._num_samples*time, time)
                    self._d_t = time
                else:
                    # If the provided argument an array we treat it as an array
                    # of time stamps
                    if len(time) != self._num_samples:
                        raise Exception("Time stamp vector length does not match the "
                                        "number of samples !!\n"
                                        ">> Num. samples: {0} | Len. time: {1} <<"
                                        .format(self._num_samples, len(time)))
                    else:
                        self._time_vec = np.asarray(time) - time[0]
                        self._d_t = np.mean(np.diff(self._time_vec))
            except Exception as e:
                print('Exception when dealing with the "time" argument:\n{0}'.format(e))
                return

            # Tau equals to the duration of the trajectory
            self.tau = self._time_vec[-1]

            # Prepare the Gaussian kernel functions
            self._prepare_gaussian_kernels()

            # Encode the DMP
            self.__encode_dmp()

    def _prepare_gaussian_kernels(self):
        self.c = np.exp(-self.a_x * np.linspace(0, 1, self.num_weights))
        self.sigma = np.square((np.diff(self.c)*0.75))
        self.sigma = np.append(self.sigma, self.sigma[-1])

    def __encode_dmp(self):
        y = self._pos_training_data

        dy = np.zeros(y.shape)
        if not(len(self._vel_training_data)):
            # Estimate the velocities if needed
            for i in range(self._num_dof):
                dy[:, i] = np.divide(np.gradient(y[:, i]), np.gradient(self._time_vec))
        else:
            dy = self._vel_training_data

        # Estimate the accelerations
        ddy = np.zeros(dy.shape)
        for i in range(self._num_dof):
            ddy[:, i] = np.divide(np.gradient(dy[:, i]), np.gradient(self._time_vec))

        # Prepare empty matrices
        ft = np.zeros((self._num_samples, self._num_dof), dtype=np.float32)
        A = np.zeros((self._num_samples, self.num_weights), dtype=np.float32)

        # Define the phase vector
        x = np.exp(-self.a_x * self._time_vec / self.tau)

        # Estimate the forcing term
        for dof in range(self._num_dof):
            ft[:, dof] = ddy[:, dof]*np.square(self.tau) - \
                self.a_z * (self.b_z * (y[-1][dof] - y[:, dof]) - dy[:, dof] * self.tau)

        for i in range(self._num_samples):
            psi = np.exp(np.divide(-0.5 * np.square(x[i] - self.c), self.sigma))
            A[i, :] = x[i] * np.divide(psi, np.sum(psi))

        # Do linear regression in the least square sense
        self.weights_pos = np.transpose(np.linalg.lstsq(A, ft)[0])

    def __decode_dmp(self):
        pass

    def _integrate_step(self, x, y, z):
        # Phase variable
        # dx = (-a_x * x) / tau
        # x = x + dx * dt
        dx = -self.a_x * x / self.tau
        x = x + dx * self._d_t

        # The weighted sum of the locally weighted regression models
        # psi = exp(-(x - c)^2 / (2 * sigma))
        psi = np.exp(- np.square(x-self.c) /
                     (np.multiply(self.sigma, 2)))

        for dof in range(self._num_dof):
            # Forcing function
            # sum( (w(dof) * x) * psi/sum(psi) )
            fx = sum(np.multiply(
                (np.multiply(self.weights_pos[dof], x)),
                (np.divide(psi, sum(psi)))
            ))

            # Derivatives
            # dz = a_z * (a_z/4 * (goal - y) - z) + fx
            dz = self.a_z * \
                (self.b_z *
                    (self.goal[dof] - y[dof])
                    - z[dof]) + fx
            dy = z[dof]

            # Temporal scaling
            dz = dz / self.tau
            dy = dy / self.tau

            # Integration
            z[dof] = z[dof] + dz * self._d_t
            y[dof] = y[dof] + dy * self._d_t

        return x, y, z

    def decode(self):
        """Function decodes the DMP and returns a trajectory.

        Returns
        ----------
        traj : np.array
            The decoded trajectory
        t : np.array
            The time samples for the decoded trajectory
        """
        # Initial states
        y = np.asarray(self.y0)
        z = np.zeros(self._num_dof)
        x = 1

        # Set a limit for the phase
        self.x_min = np.exp(-self.a_x)

        # First sample equals y0
        traj = [y]
        t = [0]

        # Decode loop
        while x > self.x_min:
            [x, y, z] = self._integrate_step(x, y, z)
            traj.append(tuple(y))
            t.append(t[-1]+self._d_t)

        traj = np.asarray(traj)
        return traj, t

    def step_decode(self):
        pass


class CartDMP(DMP):
    """CartDMP Class.

    This class implements the Dynamic Movement Primitives - DMP in Cartesian
    space. The class supports both Encoding and Decoding of trajectories.

    Attributes
    ----------
    num_weights : int
        Number of DMP weights.
    a_x : float
        The a_x gain of the DMP.
    a_z : float
        The a_z gain of the DMP.
    b_z : float
        The b_z gain of the DMP.
    c : array like
        The width of the Gaussian bell curves.
    y0 : array like
        The initial position of the DMP
    goal : array like
        The final position of the DMP
    sigma : array of floats
        The values of the Gaussian bell curves.
    num_dof : int
        The number of weights used for estimating the DMP.
    weights_pos : array of floats
        The calulated weights of the DMP for the positional part.
    weights_rot : array of floats
        The calulated weights of the DMP for the rotational part.

    Parameters
    ----------
    traj_samples : np.array or float list
        Data points of the Cartesian space trajectory to encode. Please note that
        the orientation at this point is implemented only with quaternions. This
        that the a sample point of the trajectory should have 7 entries.
    time : np.array or float list or float
        If the provided time is an array of floats with the same length as the
        `traj_samples` parameter, it will be treated as the time samples of the trajectory.
        In case this parameter is a single float, it will be treated as the sample
        time of the trajectory and the time vector will be computed accordingly.
    num_weights : int, optional
        The desired number of weights to estimate the DMP.
    a_z : float, optional
        The a_z gain of the DMP.
    a_x : float, optional
        the a_x gain of the DMP.
    reallast : bool
        Set to true if the quaternion's real part is the last element of the
        vector. Default value is false.
    """

    def __init__(self,
                 traj_samples,
                 time=None,
                 vel_data=[],
                 num_weights=25,
                 a_z=48.0,
                 a_x=2.0,
                 reallast=False):

        traj_pos = []
        traj_rot = []
        if traj_samples is not None:
            for traj_sample in traj_samples:
                traj_pos.append(traj_sample[0:3])
                traj_rot.append(traj_sample[3:])

        # Inherit the DMP class and encode the positional part of the trajectory
        super(CartDMP, self).__init__(traj_pos,
                                      time,
                                      num_weights=num_weights,
                                      a_z=a_z,
                                      a_x=a_x)

        self.__encode_quaternion_dmp(traj_rot, reallast=reallast)

    def __encode_quaternion_dmp(self, orienation_samples, reallast=False):

        # If the user says the real part of the quaternion is on the last element
        # of the vector we have to order it so it is compliant with np.quaternion
        q = np.empty(len(orienation_samples), dtype=np.quaternion)

        if reallast:
            for i, quat in enumerate(orienation_samples):
                q[i] = quaternion.from_float_array([quat[-1]] + quat[0:3])
                # q[i] = [quat[-1]] + quat[0:3]
        else:
            q = quaternion.from_float_array(orienation_samples)

        fix_quaternion_sign(q)
        # fix_quaternion_sign(q_dbg)

        # Set the goal attribute
        self.q0 = q[0]
        self.q_goal = q[-1]

        # Calculate a quaternion derivative which is needed to estimate the
        # rotation velocity
        dq = np.zeros(((np.max(q.shape), 4)), dtype=np.float32)
        for i in range(4):
            dq[:, i] = np.divide(
                np.gradient(quaternion.as_float_array(q)[:, i]),
                np.gradient(self._time_vec))

        dq = quaternion.from_float_array(dq)

        omega = np.empty((0, 3), dtype=np.float32)
        for i in range(self._num_samples):
            omega = np.append(
                omega,
                [2 * (dq[i] * q[i].conj()).imag],
                axis=0)

        # Calculate the rotation acceleration
        domega = np.empty(omega.shape, dtype=np.float32)
        for i in range(3):
            domega[:, i] = np.divide(np.gradient(omega[:, i]),
                                     np.gradient(self._time_vec))

        # Prepare empty matrices
        ft = np.zeros((self._num_samples, 3), dtype=np.float32)
        A = np.zeros((self._num_samples, self.num_weights), dtype=np.float32)
        x = np.exp(-self.a_x * self._time_vec / self.tau)

        # Estimate the forcing term
        for i in range(self._num_samples):
            ft[i, :] = np.square(self.tau) * domega[i, :] \
                + self.a_z * self.tau * omega[i, :] \
                - self.a_z * self.b_z * 2 * np.log(q[-1] * q[i].conj()).imag

            psi = np.exp(np.divide(-0.5 * np.square(x[i] - self.c), self.sigma))
            A[i, :] = x[i] * np.divide(psi, np.sum(psi))

        # Do linear regression in the least square sense
        self.weights_rot = np.linalg.lstsq(A, ft)[0].T

    def decode(self):
        pos_traj, t = super(CartDMP, self).decode()
        rot_traj, _ = self.__decode_quaterion_dmp()
        return pos_traj, np.asarray(rot_traj), t

    def __decode_quaterion_dmp(self):

        # Initial states
        y = self.q0
        z = np.quaternion(0, 0, 0, 0)
        x = 1

        # Set a limit for the phase
        self.x_min = np.exp(-self.a_x)

        # First sample equals y0
        traj = [tuple(quaternion.as_float_array(y))]
        t = [0]

        # Decode loop
        while x > self.x_min:
            [x, y, z] = self.__integrate_step_quaternion(x, y, z)

            traj.append(tuple(quaternion.as_float_array(y)))
            t.append(t[-1] + self._d_t)

        traj = np.asarray(traj)
        return traj, t

    def __integrate_step_quaternion(self, x, y, z):
        """An integration step for a quaternion DMP

        Parameters
        ----------
        x : float-type
            The current value of the DMP's phase
        y : np.quaternion
            The current state of the rotation in quaternion (y)
        z : np.quaternion
            The current values of the scaled rotation velocity (z)
        """
        # The update of the phase variable
        # dx = (-a_x * x) / tau
        # x = x + dx * dt
        dx = -self.a_x * x / self.tau
        x = x + dx * self._d_t

        # The weighted sum of the locally weighted regression models
        # psi = exp(-(x - c)^2 / (2 * sigma))
        psi = np.exp(- np.square(x-self.c) /
                     (np.multiply(self.sigma, 2)))

        fx = np.empty((3), dtype=np.float)
        for i in range(3):
            # Forcing function
            # sum( (w(i) * x) * psi/sum(psi) )
            fx[i] = sum(np.multiply(
                (np.multiply(self.weights_rot[i], x)),
                (np.divide(psi, sum(psi)))
            ))

        # Estimate the angular velocity
        dz = self.a_z * \
            (self.b_z * 2 * np.log(self.q_goal * y.conj()).imag - z.imag) +\
            fx

        # Temporal scaling
        dz = dz / self.tau

        # Integration step for the scaled velocity
        z.imag = z.imag + dz * self._d_t

        # Integration step for the orientation
        dq = np.exp(self._d_t / 2 * z / self.tau)
        y = dq * y

        return x, y, z


def fix_quaternion_sign(quat_array):
        """Function fixes sudden changes in a recorded quaternion trajectory.
        This tends to happen because of the quaternion's property: q = -q.

        Parameters
        ----------
        quat_array : array like
            A rotational trajectory expressed in quaternions
        """
        if type(quat_array[0]) == np.quaternion:
            quat_array = quaternion.as_float_array(quat_array)

        # Fix the quaternions' signs in cases where they suddenly change (q = -q)
        sign_xchange_idx = np.where((np.linalg.norm(np.diff(quat_array, axis=0), axis=1)) > 0.5)[0]

        # In case the trajectory ends with the sign flipped, add the last index to the indices array
        if len(sign_xchange_idx) % 2 != 0:
            sign_xchange_idx = np.append(sign_xchange_idx, len(quat_array)-1)

        # Change signs where neccesarry
        for i in range(len(sign_xchange_idx))[::2]:
            start_idx = sign_xchange_idx[i]+1
            end_idx = sign_xchange_idx[i+1]+1
            quat_array[start_idx:end_idx] = np.negative(quat_array[start_idx:end_idx])

        if type(quat_array[0]) == np.quaternion:
            return quaternion.from_float_array(quat_array)
        else:
            return quat_array
