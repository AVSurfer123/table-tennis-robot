# coding: utf-8
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import numpy as np
import scipy.interpolate
from collections import namedtuple

Motion = namedtuple('Motion', 'y0 dy0 gf dgf a')

DmpParams = namedtuple('DmpParams',
                       'az ay by duration dt f_func')  # these parameters should not be changed after learning a path


def gen_canonical_fun(az, tau, dt):
    z = 1.0
    while True:
        yield z
        z -= az * dt * z / tau


def gen_centers(n_bfs, az, tau, runtime):
    dist = runtime / n_bfs
    ltime = np.linspace(0.0, runtime - dist, n_bfs)
    centers = np.exp((-az / tau) * ltime)
    return centers.reshape((1, centers.shape[0]))


def gen_inv_variances(n_bfs, centers):
    hs = np.ones((1, n_bfs))
    hs[0, :-1] = np.diff(0.65 * centers) ** 2
    hs[0, -1] = hs[0, -2]
    return 1.0 / hs


def create_weights(f_target, inv_vars, centers, zs):
    psi_track = np.exp(-inv_vars.T * (zs - centers.T) ** 2)
    num = psi_track.dot((zs * f_target).T)
    den = psi_track.dot(zs.T ** 2)
    if len(f_target.shape) == 1:
        return (num / den).reshape(centers.shape)
    else:
        return (num.T / den)  # (num_dmps, n_centers)


def f_target_to_imitate_path(ys, dys, ddys, z_gen, tau, az, ay, by, coefficients):

    if len(ys.shape) == 1:
        g = np.zeros(ys.shape)
        dg = np.zeros(ys.shape)
        ddg = np.zeros(ys.shape)
        n_samples = len(ys)
        for ind in range(n_samples):
            g[ind], dg[ind], ddg[ind] = get_g_dg_ddg_from_polynomial(next(z_gen), tau, az, coefficients)[::]
        return (tau ** 2) * ddys - ay * (by * (g - ys) + (dg - tau * dys)) - (ddg * tau ** 2)
    else:
        g = np.zeros((ys.shape[0], ys.shape[1], 1))
        dg = np.zeros((ys.shape[0], ys.shape[1], 1))
        ddg = np.zeros((ys.shape[0], ys.shape[1], 1))
        n_samples = ys.shape[1]
        for ind in range(n_samples):
            g[:, ind], dg[:, ind], ddg[:, ind] = get_g_dg_ddg_from_polynomial(next(z_gen), tau, az, coefficients)
        return (tau ** 2) * ddys - ay * (by * (g[:, :, 0] - ys) + (dg[:, :, 0] - tau * dys)) - (ddg[:, :, 0] * tau ** 2)


def gen_forcing_function(centers, inv_vars, weights):
    def f(z):
        psi = np.exp(-inv_vars.T * (z - centers.T) ** 2)
        num = z * np.dot(weights, psi)
        den = np.sum(psi)

        if np.any(np.isinf(num)) and np.any(np.isinf(den)):
            raise Exception("Infinite values")
        elif den > 0:
            return num / den
        else:
            raise Exception("sum of psi == 0")

    return f


def polynomial_coefficients(final_z, tau, az, y0, dy0, gf, dgf):
    kzf = (-tau) * (np.log(final_z) / az)
    a = np.array([[1, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0],
                  [0, 0, 2, 0, 0, 0],
                  [1, kzf, kzf ** 2, kzf ** 3, kzf ** 4, kzf ** 5],
                  [0, 1, 2 * kzf, 3 * (kzf ** 2), 4 * (kzf ** 3), 5 * (kzf ** 4)],
                  [0, 0, 2, 6 * kzf, 12 * (kzf ** 2), 20 * (kzf ** 3)]])
    ddy0, ddgf = np.zeros_like(y0), np.zeros_like(y0)
    #b = np.array([y0, dy0, ddy0, gf, dgf, ddgf])
    b = np.vstack((y0, dy0, ddy0, gf, dgf, ddgf))  # shape = (6, n)
    c = scipy.linalg.solve(a, b)
    #c = np.linalg.inv(a).dot(b)  # slower
    return c


def get_g_dg_ddg_from_polynomial(z, tau, az, coefficients):
    kz = (-tau) * (np.log(z) / az)
    ind = np.arange(0, 6).reshape((6, 1))
    g = coefficients.T.dot(kz ** ind)
    dg = (ind[1:, :].T * coefficients[1:, :].T).dot(kz ** (ind[1:, :] - 1))
    ddg = ((ind[2:, :] ** 2 - ind[2:, :]).T * coefficients[2:, :].T).dot(kz ** (ind[2:, :] - 2))
    return g, dg, ddg

def gen_dmp(ay, by, az, tau, dt, final_z, motion, coefficients, f_func):
    zg = gen_canonical_fun(az, tau, dt)
    y, dy, ddy = motion.y0, motion.dy0, np.zeros_like(motion.y0)
    v, dv = motion.dy0, np.zeros_like(motion.y0)
    eta = np.exp(motion.gf - motion.y0 + 1e-9) / np.exp(motion.a)
    #next(zg)  # just for coinciding with unit test case
    r1, r2, acc_disturbance=0.0, 0.0, 0.0


    while True:
        z = next(zg)
        if z < final_z:
            dg = motion.dgf
            g = y + (motion.dgf * dt)
            ddg = np.zeros_like(dg)
            f = np.zeros_like(dg)

        else:
            g, dg, ddg = get_g_dg_ddg_from_polynomial(z, tau, az, coefficients)
            g, dg, ddg = g.T, dg.T, ddg.T
            if len(ddy.shape) == 2:
                f = f_func(z).T
            else:
                f = f_func(z)[:, 0]


        #simpletic integration
        dv = (acc_disturbance + ((ay * (by * (g - y+r1) + tau * dg - v+r2)) + (ddg * tau ** 2) + eta * f) ) / tau
        dy = v / tau
        ddy = dv / tau

        v += dv * dt
        y += dy * dt
        sensor = yield y[0, :], dy[0, :], ddy[0, :]  # avoid need for reduction
        if sensor:
            r1=sensor[0]
            r2=sensor[1]
            acc_disturbance = sensor[2]


def dmp_from_path(ys, dt=0.001, tau=1.0, az=4.0, ay=15.0, by=15.0 / 4, n_bfs=100):
    dys = np.zeros(ys.shape)
    ddys = np.zeros(ys.shape)
    if len(ys.shape) == 1:
        n_samples = len(ys)
        dys[:-1] = np.diff(ys) / dt
        ddys[:-1] = np.diff(dys) / dt
        dys[-1] = dys[-2]
        ddys[-2:] = ddys[-3]
        y0, dy0, gf, dgf = ys[0], dys[0], ys[-1], dys[-1]
    else:
        n_samples = ys.shape[1]
        dys[:, :-1] = np.diff(ys) / dt
        ddys[:, :-1] = np.diff(dys) / dt
        dys[:, -1] = dys[:, -2]
        ddys[:, -2] = ddys[:, -3]
        ddys[:, -1] = ddys[:, -3]
        y0, dy0, gf, dgf = ys[:, 0].T, dys[:, 0].T, ys[:, -1].T, dys[:, -1].T

    runtime = dt * n_samples
    centers = gen_centers(n_bfs, az, tau, runtime)
    inv_vars = gen_inv_variances(n_bfs, centers)
    z_gen = gen_canonical_fun(az=az, tau=tau, dt=dt)
    zs = np.fromiter((next(z_gen) for _ in range(n_samples)), np.float64, n_samples)
    final_z = zs[-1]
    coefficients = polynomial_coefficients(final_z, tau, az, y0, dy0, gf, dgf)
    f_target = f_target_to_imitate_path(ys, dys, ddys, gen_canonical_fun(az=az, tau=tau, dt=dt), tau, az, ay, by, coefficients)
    weights = create_weights(f_target, inv_vars, centers, zs)
    return DmpParams(az=az, ay=ay, by=by, duration=runtime, dt=dt,
                     f_func=gen_forcing_function(centers, inv_vars, weights))


def get_dmp_motion(dmp_params, motion, duration):
    #assert motion.y0.shape == (1, 3)
    tau = duration / dmp_params.duration
    z_gen = gen_canonical_fun(dmp_params.az, tau, dmp_params.dt)
    zs = [next(z_gen) for _ in range(int(duration / dmp_params.dt))]
    coefficients = polynomial_coefficients(zs[-1], tau, dmp_params.az, motion.y0, motion.dy0, motion.gf, motion.dgf)
    return gen_dmp(dmp_params.ay, dmp_params.by, dmp_params.az, tau, dmp_params.dt, zs[-1], motion, coefficients,
                   dmp_params.f_func)


if __name__ == '__main__':

    n_basis = 100
    dt = 0.001
    movement_time = 1.0
    n_samples = int(movement_time/dt)
    t = np.arange(0,movement_time,dt)
    T=t.shape[0]

    centers = np.linspace(0,1,n_basis).reshape((1, n_basis))
    hs = np.zeros_like(centers)
    hs[0,:-1] = (np.diff(centers)*0.65)**2
    hs[0,-1] = hs[0,-2]
    hs = 1./hs

    demo = 2*t**2 + np.cos(t*4*np.pi)*(1-t) - 1;

    demo_d = np.zeros_like(demo)
    demo_dd = np.zeros_like(demo)
    demo_d[:-1] = np.diff(demo)/dt
    demo_dd[:-1] = np.diff(demo_d)/dt
    demo_d[-1]=demo_d[-2]
    demo_dd[-2:]=demo_dd[-3]

    PF = demo[-1]
    P0 = demo[0]
    V0 = demo_d[0]
    VF = demo_d[-1]


    dmp_params = dmp_from_path(demo, ay=50.0, by=50.0/4, n_bfs=n_basis, dt=dt)
    y0 = np.array([1.0*P0]).reshape(1,1)
    yf = np.array([1.0*PF]).reshape(1,1)
    dy0 = np.array([1.0*V0]).reshape(1,1)
    dyf = np.array([1.0*VF]).reshape(1,1)

    hit = Motion(y0=y0, dy0=dy0, gf=yf, dgf=dyf, a=PF-P0)
    print(hit)
    duration = 1.0
    dmp_gen = get_dmp_motion(dmp_params=dmp_params, motion=hit, duration=duration)
    out_y = np.zeros((1,n_samples))
    out_dy = np.zeros((1,n_samples))
    out_ddy = np.zeros((1,n_samples))
    for t in range(n_samples):
        ydmp, dydmp, ddydmp = next(dmp_gen)
        out_y[:,t], out_dy[:,t], out_ddy[:,t] = ydmp[:], dydmp[:], ddydmp[:]
    ts2 = np.linspace(0.0, 1.0, n_samples)



