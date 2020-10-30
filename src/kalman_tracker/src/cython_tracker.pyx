#from cython.view cimport array as cvarray
import numpy as np
cimport numpy as np

DTYPE = np.double
ctypedef np.double_t DTYPE_t


DRAG = 0.01  #kd0rag o0effii0ent o0ff0
MASS = 0.1  #m mass of projectile
#vx = 1-(k/m)*dt vx

GRAV = -9.8


cdef class Tracker:
    prediction = None

    def __init__(self, dt):
        pass

    def update(self, z_measured, dt):
        pass


cdef class LinearKF(Tracker):
    cdef np.ndarray _transition_matrix
    cdef np.ndarray _state_post
    cdef np.ndarray _state_pre
    cdef np.ndarray _measurement_matrix
    cdef np.ndarray _measurement_noise_cov
    cdef np.ndarray _process_noise_cov
    cdef np.ndarray _error_cov_post
    cdef np.ndarray _error_cov_pre

    def __init__(self, float dt):
        self._upd_transition_matrix(dt)

        self._process_noise_cov = np.array([[0.0001, 0, 0, 0, 0, 0, 0],
                                            [0, 0.0001, 0, 0, 0, 0, 0],
                                            [0, 0, 0.0001, 0, 0, 0, 0],
                                            [0, 0, 0, 100, 0, 0, 0],
                                            [0, 0, 0, 0, 100, 0, 0],
                                            [0, 0, 0, 0, 0, 100, 0],
                                            [0, 0, 0, 0, 0, 0, 0.001]])

        self._error_cov_post = np.array([[0.0001, 0, 0, 0, 0, 0, 0],
                                         [0, 0.0001, 0, 0, 0, 0, 0],
                                         [0, 0, 0.0001, 0, 0, 0, 0],
                                         [0, 0, 0, 100, 0, 0, 0],
                                         [0, 0, 0, 0, 100, 0, 0],
                                         [0, 0, 0, 0, 0, 100, 0],
                                         [0, 0, 0, 0, 0, 0, 0.001]])

        self._error_cov_pre = np.array([[0.0001, 0, 0, 0, 0, 0, 0],
                                        [0, 0.0001, 0, 0, 0, 0, 0],
                                        [0, 0, 0.0001, 0, 0, 0, 0],
                                        [0, 0, 0, 100, 0, 0, 0],
                                        [0, 0, 0, 0, 100, 0, 0],
                                        [0, 0, 0, 0, 0, 100, 0],
                                        [0, 0, 0, 0, 0, 0, 0.001]])




        self._measurement_noise_cov = (0.01 ** 2) * np.identity(3)

        self._measurement_matrix = np.zeros((3, 7))
        self._measurement_matrix[0:3, :3] = np.identity(3)

        self._state_post = np.array([[1.0, 1.0, 1.0, 0, 0, 0, GRAV]]).transpose()
        self._state_pre = np.array([[1.0, 1.0, 1.0, 0, 0, 0, GRAV]]).transpose()

    def _upd_transition_matrix(self, float dt):
        self._transition_matrix = np.array([[1, 0, 0, dt, 0, 0, 0],
                                            [0, 1, 0, 0, dt, 0, 0],
                                            [0, 0, 1, 0, 0, dt, 0.5 * dt * dt],
                                            [0, 0, 0, 1, 0, 0, 0],
                                            [0, 0, 0, 0, 1, 0, 0],
                                            [0, 0, 0, 0, 0, 1, dt],
                                            [0, 0, 0, 0, 0, 0, 1]])

    def _predict(self):
        self._state_pre = self._transition_matrix.dot(self._state_post)
        self._error_cov_pre = self._transition_matrix.dot(self._error_cov_post).dot(
            self._transition_matrix.transpose()) + self._process_noise_cov

    def update(self, np.ndarray z_measured, double dt):
        self._upd_transition_matrix(dt)
        self._predict()

        # measurement update
        Z = np.array(z_measured, ndmin=2)
        y = Z.transpose() - self._measurement_matrix.dot(self._state_pre)

        #self.error.append(np.sum(y**2))

        S = self._measurement_matrix.dot(self._error_cov_pre.dot(self._measurement_matrix.transpose())) \
            + self._measurement_noise_cov
        K = self._error_cov_pre.dot(self._measurement_matrix.transpose().dot(np.linalg.inv(S)))
        self._state_post = self._state_pre + (K.dot(y))
        self._error_cov_post = (np.identity(self._error_cov_pre.shape[0]) - (K.dot(self._measurement_matrix))).dot(
            self._error_cov_pre)

    def get_state(self, float dt=0.0):
        if dt == 0.0:
            return self._state_post[:, 0]
        else:
            self._upd_transition_matrix(dt)
            return self._transition_matrix.dot(self._state_post)