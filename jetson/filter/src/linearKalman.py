import numpy as np
from copy import deepcopy
from scipy.linalg import block_diag


class LinearKalmanFilter:
    '''
    Class for Linear Kalman filtering

    @attribute int dim_x: dimension of state space
    @attribute int dim_z: dimension of measurement space
    @attribute int dim_u: dimension of control input
    @attribute ndarray x: state vector
    @attribute ndarray P: state covariance matrix
    @attribute ndarray z: measurement vector
    @attribute ndarray R: measurement noise matrix
    @attribute ndarray Q: process noise matrix
    @attribute ndarray F: state transition matrix
    @attribute ndarray B: control transition matrix
    @attribute ndarray H: state space -> measurement space transform matrix
    @attribute ndarray K: Kalman gain
    @attribute ndarray y: residual
    @attribute ndarray _I: identity matrix
    @attribute function inv: matrix inverse function
    '''

    def __init__(self, dim_x, dim_z, dim_u=0, p_inverse=False):
        '''
        Initializes filter matrices

        @param int dim_x: dimension of state space
        @param int dim_z: dimension of measurement space
        @optional int dim_u: dimension of control input
        @optional bool p_inverse: use Moore-Penrose pseudo-inverse
        '''
        if dim_x < 1:
            raise ValueError('dim_x must be 1 or greater')
        if dim_z < 1:
            raise ValueError('dim_z must be 1 or greater')
        if dim_u < 0:
            raise ValueError('dim_u must be 0 or greater')

        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dim_u = dim_u

        self.x = np.zeros((dim_x, 1))
        self.P = np.eye(dim_x)
        self.Q = np.eye(dim_x)
        self.B = None
        self.F = np.eye(dim_x)
        self.H = np.zeros((dim_z, dim_x))
        self.R = np.eye(dim_z)

        self.K = np.zeros((dim_x, dim_z))
        self.y = np.zeros((dim_z, 1))

        self._I = np.eye(dim_x)

        self.inv = np.linalg.pinv if p_inverse else np.linalg.inv

    def construct(self, x_initial, P_initial, F, H, Q, R, B=None):
        '''
        Constructs the LKF using initial values

        @param StateEstimate x_initial: initial state estimate
        @param list P_initial: initial variance values for each state variable
        @param ndarray F: state transition matrix
        @param ndarray H: state space -> measurement space transform matrix
        @param ndarray Q: process noise matrix
        @param list R: measurement noise for each measurement variable
        @param optional B: control transition matrix
        '''

        x_initial = x_initial.asLKFInput()
        x_initial = np.atleast_2d(x_initial)
        if x_initial.shape == (1, self.dim_x):
            x_initial = x_initial.T
        if x_initial.shape != (self.dim_x, 1):
            raise ValueError("x_initial dimension mismatch")
        self.x = x_initial

        if len(P_initial) != self.dim_x:
            raise ValueError("P_initial dimension mismatch")
        self.P[:] = np.diag(P_initial)

        if len(R) != self.dim_z:
            raise ValueError("R dimension mismatch")
        self.R[:] = np.diag(R)

        self.F = deepcopy(F)
        self.B = deepcopy(B)
        self.H = deepcopy(H)
        self.Q = deepcopy(Q)

    def predict(self, u=None, B=None, F=None, Q=None):
        '''
        Predict forward to get priors

        @optional ndarray u: control input vector
        @optional ndarray B: control transition matrix
        @optional ndarray F: state transition matrix
        @optional float/ndarray Q: process noise matrix
        '''

        if B is None:
            B = self.B
        if F is None:
            F = self.F
        if Q is None:
            Q = self.Q
        elif np.isscalar(Q):
            Q = np.eye(self.dim_x) * Q

        # State prediction: x = Fx + Bu
        if B is not None and u is not None:
            u = np.atleast_2d(u)
            if u.shape == (1, self.dim_u):
                u = u.T
            if u.shape != (self.dim_u, 1):
                raise ValueError("u dimension mismatch")
            self.x = np.dot(F, self.x) + np.dot(B, u)
        else:
            self.x = np.dot(F, self.x)

        # Covariance prediction: P = FPF' + Q
        self.P = np.dot(np.dot(F, self.P), F.T) + Q

    def update(self, z, R=None, H=None):
        '''
        Update the filter with a measurement

        @param ndarray z: measurement vector
        @optional float/list R: measurement noise for each measurement variable
        @optional ndarray H: state space -> measurement space transform matrix
        '''

        if z is None:
            self.y = np.zeros((self.dim_z, 1))
            return

        z = np.atleast_2d(z)
        if z.shape == (1, self.dim_z):
            z = z.T
        if z.shape != (self.dim_z, 1):
            raise ValueError("z dimension mismatch")
        # z = reshape_z(z, self.dim_z, self.x.ndim)

        if R is None:
            R = self.R
        elif np.isscalar(R):
            R = np.eye(self.dim_z) * R
        else:
            R = np.diag(R)

        if H is None:
            H = self.H

        # Residual: y = z - Hx
        self.y = z - np.dot(H, self.x)

        PHT = np.dot(self.P, H.T)
        # System uncertainty in measurement space: S = HPH' + R
        S = np.dot(H, PHT) + R
        # Kalman gain: K = PH'/S
        S_inverse = self.inv(S)
        self.K = np.dot(PHT, S_inverse)

        # State estimate: x = x + Ky
        self.x = self.x + np.dot(self.K, self.y)

        # Covariance estimate: P = (I-KH)P(I-KH)' + KRK'
        # P = (I-KH)P usually seen in the literature
        # "This is more numerically stable and works for non-optimal K vs the equation" - filterpy
        I_KH = self._I - np.dot(self.K, H)
        self.P = np.dot(np.dot(I_KH, self.P), I_KH.T) + np.dot(np.dot(self.K, R), self.K.T)


def QDiscreteWhiteNoise(dim, dt, variance=1., block_size=1):
    '''
    Returns the Q matrix for the Discrete Constant White Noise Model.
    Q is computed as the G * G^T * variance, where G is the process noise per time step

    Copied from filterpy

    @param int dim: dimensions in Q
    @param float dt: timestep
    @optional float variance: variance in noise
    @optional int block_size: number of derivatives in one dimension
    @return ndarray: white noise matrix
    '''

    if not (dim == 2 or dim == 3 or dim == 4):
        raise ValueError("dim must be between 2 and 4")

    if dim == 2:
        Q = [[.25*dt**4, .5*dt**3],
             [.5*dt**3, dt**2]]
    elif dim == 3:
        Q = [[.25*dt**4, .5*dt**3, .5*dt**2],
             [.5*dt**3, dt**2, dt],
             [.5*dt**2, dt, 1]]
    else:
        Q = [[(dt**6)/36, (dt**5)/12, (dt**4)/6, (dt**3)/6],
             [(dt**5)/12, (dt**4)/4, (dt**3)/2, (dt**2)/2],
             [(dt**4)/6, (dt**3)/2, dt**2, dt],
             [(dt**3)/6, (dt**2)/2, dt, 1.]]

    return block_diag(*[Q]*block_size) * variance
