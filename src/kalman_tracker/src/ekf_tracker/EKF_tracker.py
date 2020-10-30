from __future__ import division
import numpy as np

import pyximport; pyximport.install()
import cython_funcs

class Tracker:
    prediction = None

    def __init__(self, dt):
        raise NotImplementedError()

    def update(self, z_measured, dt):
        raise NotImplementedError()

    def get_state(self, dt=0.0):
        raise NotImplementedError()

R_COEF_TABLE = np.array([0.73, 0.73, -0.92])  # restitution coefficient of table (modificar tambem no no de trajectoria)
GRAV_ACC = -9.8

CW_DRAG_COEF = 0.47
P_AIR_DENS = 1.293  # [Kg/m^3]
A_SECTION = 0.0013
BALL_MASS = 0.0027

GROUND_Z = 0.025

MHD_THRESHOLD = 100
NUMBER_INI_SAMPLES = 5
MAX_REJECTED_SAMPLES = 5
#ddx_k+1 = g - Cw*p*A/2m * norm(dx_k)*dx_k
#dx_k+1 = dx_k + ddx_k+1 * dt
#x_k+1 = x_k + dx_k+1 * dt
class EkfTracker(Tracker):
    _f = None  # used to store the computed transition matrix for each step
    _h = None  # used to store the computed measurement matrix for each step
    _F = None  # jacobian of the transition function (dF/dx)_k-1 | x_k-1|x_k-1, u_k-1
    _x_post = None  # State x_k|k-1
    _x_pre = None  # State x_k-1|k-1
    _H = None  # jacobian of Measurement function (dh/dx)
    _Q = None  # process noise covariance Qk
    _P_post = None  # error covariance P_k|k-1
    _P_pre = None  # error covariance Pk-1|k-1
    _R = None  # measurement noise covariance R_k

    _must_reset = True
    _accumulated_initialization_samples = np.zeros((3, NUMBER_INI_SAMPLES))
    _number_of_accumulated_initialization_samples = 0
    _rejected_samples = 0

    def __init__(self, dt):
        self._f = np.zeros((9, 9))
        self._h = np.zeros((3, 3))
        self._x_post = np.zeros(9)
        self._x_pre = np.zeros(9)
        self._P_post = (1.0e5 ** 2) * np.identity(9)
        self._P_pre = (1.0e5 ** 2) * np.identity(9)
        self._F = np.zeros((9, 9))
        self._Q = (0.001 ** 2) * np.identity(9)
        self._R = (0.001 ** 2) * np.identity(3)

        self._h = np.zeros((3, 9))
        self._h[0:3, :3] = np.identity(3)  # measurement matrix is linear
        self._H = self._h  # jacobian is the measurement matrix
        self._must_reset = True

    @staticmethod
    def f_fun(dt, x_pre_modified, x_post_unchanged, F_modified=None):
        den = np.linalg.norm(x_post_unchanged[3:6]) #optimization since norm is used twice
        #ddx
        x_pre_modified[6:9] = np.array([0, 0, GRAV_ACC]) - ((CW_DRAG_COEF * P_AIR_DENS * A_SECTION) / (2 * BALL_MASS)) * den * x_post_unchanged[3:6]
        x_pre_modified[3:6] = x_post_unchanged[3:6] + dt*x_pre_modified[6:9]  # dx = dx +dt*ddx
        x_pre_modified[0:3] = x_post_unchanged[0:3] + dt*x_pre_modified[3:6]  # x = x + dt*dx

        if F_modified is not None:
            np.fill_diagonal(F_modified[0:3, 0:3], 1)
            np.fill_diagonal(F_modified[3:6, 3:6], 1)
            np.fill_diagonal(F_modified[0:3, 3:6], dt)
            np.fill_diagonal(F_modified[3:6, 6:9], dt)
            K = -((CW_DRAG_COEF * P_AIR_DENS * A_SECTION) / (2 * BALL_MASS))
            val = np.sum(x_post_unchanged[0:3]**2) + (x_post_unchanged[3:6]**2)
            #den = np.linalg.norm(x_post[3:6]) # calculated previously
            if den != 0.0:
                val *= (K/den)
            else:
                val = 0
            np.fill_diagonal(F_modified[6:9, 3:6], val)  # d/d (dx) (-K*sqrt(dx^2+dy^2+dz^2)*[dx,dy,dz]

    def h_fun(self, x_pre):
        return self._h.dot(x_pre)

    #@profile
    def _predict(self, dt=1.0/120.0):
        #self.f_fun(dt, self._x_pre, self._x_post, F_modified=self._F)  # sets self._x_pre
        cython_funcs.f_fun(dt, self._x_pre, self._x_post, F_modified=self._F)  # sets self._x_pre
        if self._x_pre[2] <= GROUND_Z:
            self._x_post[3:6] = R_COEF_TABLE * self._x_post[3:6]
            #self.f_fun(dt, self._x_pre, self._x_post, F_modified=self._F)
            cython_funcs.f_fun(dt, self._x_pre, self._x_post, F_modified=self._F)

        if not np.isfinite(np.sum(self._x_pre)):
            self._must_reset = True
            self._x_pre = np.zeros_like(self._x_pre)
            self._x_post = np.zeros_like(self._x_pre)

        self._P_pre = self._F.dot(self._P_post).dot(self._F.T) + self._Q

    def initialize(self, z):
        if self._must_reset:
            self._accumulated_initialization_samples[:,
                self._number_of_accumulated_initialization_samples] = z
            self._number_of_accumulated_initialization_samples += 1
            if self._number_of_accumulated_initialization_samples >= NUMBER_INI_SAMPLES:
                dts = np.arange(0.0, NUMBER_INI_SAMPLES*(1.0/120.0), 1.0/120.0)
                px = np.polyfit(dts, self._accumulated_initialization_samples[0, :], 1)
                py = np.polyfit(dts, self._accumulated_initialization_samples[1, :], 1)
                pz = np.polyfit(dts, self._accumulated_initialization_samples[2, :], 1)

                self._x_pre[:] = (z[0], z[1], z[2], px[0], py[0], pz[0], 0, 0, 0)
                self._x_post[:] = (z[0], z[1], z[2], px[0], py[0], pz[0], 0, 0, 0)
                self._must_reset = False
                self._rejected_samples = 0
                self._number_of_accumulated_initialization_samples = 0
            else:
                return None

    def passes_validation_gate(self, S, y):
        """Validation Gate, if residual error too large is an outlier, best if done outside tracker"""
        g = MHD_THRESHOLD
        try:
            mhd = y.T.dot(np.linalg.inv(S).dot(y))
        except np.linalg.LinAlgError:
            raise Exception('EKF became unstable, increase added noise?')
        if mhd > g ** 2:
            self._rejected_samples += 1
            if self._rejected_samples > 4:  # More than 4 outliers reset filter on next sample
                self._must_reset = True
            # since sample is not used set state as only the prediction
            self._x_post = self._x_pre
            self._P_post = self._P_pre
            return False
        return True


    def update(self, z_measurement, dt=1.0/120.0):
        z_measurement += 1e-9*np.random.random(z_measurement.shape)  # important to avoid instability of EKF
        self._predict()

        if np.isnan(np.sum(z_measurement)):
            self._rejected_samples += 1
            if self._rejected_samples > MAX_REJECTED_SAMPLES:
                self._must_reset = True
            return np.nan
        elif self._must_reset:
            self.initialize(z_measurement)
            return np.nan
        else:
            y = z_measurement - self.h_fun(self._x_pre)
            S = self._H.dot(self._P_pre).dot(self._H.T) + self._R

            if self.passes_validation_gate(S, y):
                K = self._P_pre.dot(self._H.T).dot(np.linalg.inv(S))
                self._x_post = self._x_pre + K.dot(y)
                self._P_post = (np.identity(9)-K.dot(self._H)).dot(self._P_pre)
                self._rejected_samples = 0
            return y.T.dot(np.linalg.inv(S).dot(y))  # return error if passes or not valid gate

    def get_state(self, dt=0.0):
        if self._must_reset:
            nans = np.empty(self._x_post.shape)
            nans.fill(np.NaN)
            return nans

        if dt == 0.0:
            return self._x_post[:]
        else:
            time_step = 1.0 / 120.0
            rc_table = R_COEF_TABLE
            state_pre = self._x_pre
            res = np.zeros(9)
            for i in range(int(dt / time_step)):
                self.f_fun(time_step, res, state_pre, F_modified=self._F)
                if res[2] <= GROUND_Z:
                    res[3:6] = rc_table * res[3:6]
                    state_pre = res
                    self.f_fun(time_step, res, state_pre, F_modified=self._F)
                state_pre = res
            return res


if __name__ == '__main__':
    nan = np.NaN
    KF = EkfTracker(1.0/120.0)
    x, y, z, vx, vy, vz, ax, ay, az = KF.get_state()
    test = np.array([[        nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan, -0.46712823,
        -0.46530306, -0.46317109, -0.46096926, -0.45895499, -0.45764935,
        -0.45725844, -0.45730712, -0.45736713, -0.45752189, -0.4577544 ,
        -0.4580648 , -0.45837487, -0.45872569, -0.45909011, -0.45949258,
        -0.45990423, -0.46033349, -0.46079028, -0.46117546, -0.46159172,
        -0.4620703 , -0.46253738, -0.46304016, -0.46352274, -0.46395887,
        -0.46447948, -0.46498637, -0.46553881, -0.46601538, -0.46653736,
        -0.46699592, -0.4676063 , -0.46803116, -0.46867011, -0.46917922,
        -0.46972018, -0.47027221, -0.47083556, -0.47136432, -0.47192865,
        -0.47248596, -0.47302508, -0.47356793, -0.47411942, -0.47465526,
        -0.47520651, -0.47574179, -0.4762831 , -0.47686061, -0.47746473,
        -0.47807722, -0.47869643, -0.47931153, -0.47991223, -0.4805004 ,
        -0.48111795, -0.48169121, -0.48280795, -0.48292476,         nan,
                nan, -0.48479157, -0.48534542, -0.48590949, -0.48647577,
        -0.48711563, -0.48777722, -0.48848224, -0.48914417, -0.48981877,
        -0.49049147, -0.49120494, -0.49191145, -0.49264085, -0.49336988],
       [        nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan, -1.78919093,
        -1.77322412, -1.75495646, -1.73412562, -1.71129072, -1.68701224,
        -1.66236834, -1.63807854, -1.61393805, -1.58989996, -1.56588017,
        -1.54194047, -1.5180904 , -1.49436153, -1.4707643 , -1.44723739,
        -1.42382887, -1.40049258, -1.37734857, -1.35415578, -1.3311333 ,
        -1.3080945 , -1.28511288, -1.26218321, -1.23927773, -1.21640235,
        -1.19357054, -1.17081373, -1.14807894, -1.12540304, -1.10279319,
        -1.08031606, -1.05794234, -1.03553639, -1.01321091, -0.99087496,
        -0.96863184, -0.94646536, -0.92440107, -0.90238113, -0.88038663,
        -0.85845439, -0.8365932 , -0.81478997, -0.79301064, -0.77131498,
        -0.749676  , -0.72809448, -0.70652923, -0.68497827, -0.66346791,
        -0.64205772, -0.62070179, -0.59938568, -0.57811182, -0.55689434,
        -0.53573789, -0.5146654 , -0.09189696, -0.08837284,         nan,
                nan, -0.41021582, -0.38956364, -0.36896287, -0.34844784,
        -0.32794837, -0.30750497, -0.28709996, -0.2668115 , -0.24660832,
        -0.22644316, -0.2063884 , -0.1863308 , -0.16639424, -0.14649662],
       [        nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,         nan,
                nan,         nan,         nan,         nan,  0.02867398,
         0.03847359,  0.05053941,  0.06569517,  0.08425394,  0.10626522,
         0.13031405,  0.15453066,  0.1781142 ,  0.2008384 ,  0.22290367,
         0.24421831,  0.26487061,  0.284769  ,  0.30385715,  0.32225874,
         0.33992132,  0.35691254,  0.37313297,  0.38877386,  0.40365869,
         0.41785839,  0.43129374,  0.44398966,  0.45600439,  0.46729057,
         0.47789287,  0.48776625,  0.49695014,  0.50543756,  0.51321143,
         0.52026753,  0.52660354,  0.53231452,  0.53725889,  0.54153423,
         0.54513832,  0.54796266,  0.55017761,  0.55172156,  0.55253199,
         0.55263364,  0.55211273,  0.55096016,  0.54916826,  0.54666507,
         0.54345492,  0.53958625,  0.53503608,  0.52984355,  0.52403602,
         0.51761971,  0.51052431,  0.50277598,  0.49437346,  0.4852892 ,
         0.47555499,  0.46516566,  0.02614441,  0.02942418,         nan,
                nan,  0.40326002,  0.38898057,  0.37398209,  0.35839147,
         0.34216229,  0.32530595,  0.3077966 ,  0.28958179,  0.27083455,
         0.2513952 ,  0.23136763,  0.21068568,  0.18944768,  0.16751146]])

    test2 = np.zeros((3,500))
    test2[0,:] = -0.5
    test2[1,:] = 7.0
    test2[2,:] = 0.02
    #test2 += 1e-9 * np.random.random(test2.shape)

    for i in range(test2.shape[1]):
        z = test2[:, i]
        KF.update(z, 1.0 / 120)
        #print(KF.get_state(0.5))
    x, y, z, vx, vy, vz, ax, ay, az = KF.get_state()
    pass