import numpy as np

GRAV = -9.8
R_COEF_TABLE = np.array([0.73, 0.73, -0.92])  # restitution coefficient of table


class Tracker:
    prediction = None

    def __init__(self, dt):
        raise NotImplementedError()

    def update(self, z_measured, dt):
        raise NotImplementedError()

    def get_state(self, dt=0.0):
        raise NotImplementedError()


class LinearKF(Tracker):
    _transition_matrix = None
    _state_post = None
    _state_pre = None
    _measurement_matrix = None
    _measurement_noise_cov = None
    _process_noise_cov = None
    _error_cov_post = None
    _error_cov_pre = None

    _must_reset = True
    _accumulated_initialization_samples = np.zeros((3, 10))
    _number_of_accumulated_initialization_samples = 0
    _rejected_samples = 0

    def __init__(self, dt):
        self._upd_transition_matrix(dt)

        #"""
        self._process_noise_cov = np.array([[0.001, 0, 0, 0, 0, 0, 0],
                                            [0, 0.001, 0, 0, 0, 0, 0],
                                            [0, 0, 0.001, 0, 0, 0, 0],
                                            [0, 0, 0, 0.01, 0, 0, 0],
                                            [0, 0, 0, 0, 0.01, 0, 0],
                                            [0, 0, 0, 0, 0, 0.01, 0],
                                            [0, 0, 0, 0, 0, 0, 0.001]])
        #"""
        #self._process_noise_cov = np.zeros((7, 7))

        self._error_cov_post = np.array([[0.0001, 0, 0, 0, 0, 0, 0],
                                         [0, 0.0001, 0, 0, 0, 0, 0],
                                         [0, 0, 0.0001, 0, 0, 0, 0],
                                         [0, 0, 0, 10, 0, 0, 0],
                                         [0, 0, 0, 0, 10, 0, 0],
                                         [0, 0, 0, 0, 0, 10, 0],
                                         [0, 0, 0, 0, 0, 0, 0.001]])

        self._error_cov_pre = np.array([[0.0001, 0, 0, 0, 0, 0, 0],
                                        [0, 0.0001, 0, 0, 0, 0, 0],
                                        [0, 0, 0.0001, 0, 0, 0, 0],
                                        [0, 0, 0, 1, 0, 0, 0],
                                        [0, 0, 0, 0, 1, 0, 0],
                                        [0, 0, 0, 0, 0, 1, 0],
                                        [0, 0, 0, 0, 0, 0, 0.001]])

        self._measurement_noise_cov = (0.005 ** 2) * np.identity(3)

        self._measurement_matrix = np.zeros((3, 7))
        self._measurement_matrix[0:3, :3] = np.identity(3)

        self._state_post = np.array([[1.0, 1.0, 1.0, 0, 0, 0, GRAV]]).transpose()
        self._state_pre = np.array([[1.0, 1.0, 1.0, 0, 0, 0, GRAV]]).transpose()

    def _upd_transition_matrix(self, dt):
        self._transition_matrix = np.array([[1, 0, 0, dt, 0, 0, 0],
                                            [0, 1, 0, 0, dt, 0, 0],
                                            [0, 0, 1, 0, 0, dt, 0.5 * dt * dt],
                                            [0, 0, 0, 1, 0, 0, 0],
                                            [0, 0, 0, 0, 1, 0, 0],
                                            [0, 0, 0, 0, 0, 1, dt],
                                            [0, 0, 0, 0, 0, 0, 1]])

    def set_state_pre_post(self, x, y, z, vx, vy, vz, az):
        self._state_pre[:, 0] = self._state_post[:, 0] = (x, y, z, vx, vy, vz, az)

    def _predict(self):

        res = self._transition_matrix.dot(self._state_post)
        if res[2, 0] <= 0.025:
            res[3:6, 0] = R_COEF_TABLE * res[3:6, 0]
        self._state_pre = res
        #self._state_pre = self._transition_matrix.dot(self._state_post)
        self._error_cov_pre = self._transition_matrix.dot(self._error_cov_post).dot(
            self._transition_matrix.transpose()) + self._process_noise_cov

    def update(self, z_measured, dt):

        #check if measurement is valid
        z_measured = np.array(z_measured, ndmin=2)
        valid_z_measured = np.isfinite(z_measured).all() and z_measured.shape == (1, 3)

        # reset when following a new ball launch (discontinuity)
        if self._must_reset and valid_z_measured:
            self._accumulated_initialization_samples[:,
                self._number_of_accumulated_initialization_samples] = z_measured
            self._number_of_accumulated_initialization_samples += 1
            if self._number_of_accumulated_initialization_samples > 9:
                dts = np.arange(0.0, 10*(1.0/120.0), 1.0/120.0)
                px = np.polyfit(dts, self._accumulated_initialization_samples[0, :], 1)
                py = np.polyfit(dts, self._accumulated_initialization_samples[1, :], 1)
                pz = np.polyfit(dts, self._accumulated_initialization_samples[2, :], 1)

                self.set_state_pre_post(z_measured[0, 0], z_measured[0, 1], z_measured[0, 2], px[0], py[0], pz[0], GRAV)
                self._must_reset = False
                self._number_of_accumulated_initialization_samples = 0
            else:
                return None

        #KF prediction step
        self._upd_transition_matrix(dt)
        self._predict()

        if not valid_z_measured:
            return np.nan

        else:
            # measurement update
            Z = z_measured
            y = Z.transpose() - self._measurement_matrix.dot(self._state_pre)

            #self.error.append(np.sum(y**2))

            S = self._measurement_matrix.dot(self._error_cov_pre.dot(self._measurement_matrix.transpose())) \
                + self._measurement_noise_cov

            #Validation Gate, if residual error too large is an outlier, best if done outside tracker
            g = 10
            if y.T.dot(np.linalg.inv(S).dot(y)) > g ** 2:
                self._rejected_samples += 1
                if self._rejected_samples > 4:  # More than 4 outliers reset filter on next sample
                    self._must_reset = True

                # since sample is not used set state as only the prediction
                self._state_post = self._state_pre
                self._error_cov_post = self._error_cov_pre
                return None

            else:  # sample is valid and not an outlier perform update step
                self._rejected_samples = 0  # Sample is not rejected by validation gate

                K = self._error_cov_pre.dot(self._measurement_matrix.transpose().dot(np.linalg.inv(S)))
                self._state_post = self._state_pre + (K.dot(y))
                self._error_cov_post = (
                    np.identity(self._error_cov_pre.shape[0]) - (K.dot(self._measurement_matrix))).dot(
                    self._error_cov_pre)

                return y.T.dot(np.linalg.inv(S).dot(y))

    def get_state(self, dt=0.0):
        if self._must_reset:
            nans = np.empty(self._state_post.shape)
            nans.fill(np.NaN)
            return nans

        if dt == 0.0:
            return self._state_post[:, 0]
        else:
            time_step = 1.0 / 120.0
            rc_table = R_COEF_TABLE
            state_pre = self._state_pre
            self._upd_transition_matrix(time_step)
            for i in range(int(dt / time_step)):
                res = self._transition_matrix.dot(state_pre)
                #error_cov = self._transition_matrix.dot(self._error_cov_pre).dot(
                #    self._transition_matrix.T) + self._process_noise_cov
                if res[2, 0] <= 0.025:
                    res[3:6, 0] = rc_table * res[3:6, 0]
                state_pre = res
            return res


if __name__ == '__main__':
    KF = LinearKF(1.0 / 120)
    x, y, z, vx, vy, vz, az = KF.get_state()
    KF.update([0.1, 0.1, 0.1], 1.0 / 120)
    KF.predict_ahead(0.25)
    x, y, z, vx, vy, vz, az = KF.get_state()
