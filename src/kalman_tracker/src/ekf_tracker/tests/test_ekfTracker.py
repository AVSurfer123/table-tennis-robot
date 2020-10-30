from unittest import TestCase
import numpy as np
from ekf_tracker.EKF_tracker import EkfTracker

import pyximport; pyximport.install()
import ekf_tracker.cython_funcs

class TestEkfTracker(TestCase):
    def test_f_fun(self):

        dt = 1.0/120
        x_pre = np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0])
        x_post = np.copy(x_pre)
        F = np.zeros((9, 9))
        F2 = np.zeros((9, 9))
        EkfTracker.f_fun(dt, x_pre, x_post, F)
        correct = np.array([5.00833157e-01, 5.00833157e-01, 5.00152602e-01, 9.99788833e-02,
                            9.99788833e-02, 1.83122167e-02, -2.53399995e-03, -2.53399995e-03,
                            -9.80253400e+00])
        self.assertTrue(np.allclose(x_pre, correct, 1e-7))
        ekf_tracker.cython_funcs.f_fun(dt, x_pre, x_post, F2)
        self.assertTrue(np.allclose(x_pre, correct, 1e-7))
        self.assertTrue(np.allclose(F, F2, 1e-7))
