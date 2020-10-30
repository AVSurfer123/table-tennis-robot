#Imitation of motion primitives by global optimization
import numpy as np
import cvxopt as cvx
from cvxopt import solvers
from cvxopt import spmatrix
from multiprocessing import Pool
import scipy.interpolate
import itertools
#import mosek
#solvers.options['solver'] = 'mosek'
import pylab

N_CPU = 1

#Low Pass
#!python
def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    r"""Smooth (and optionally differentiate) data with a Savitzky-Golay filter.
    The Savitzky-Golay filter removes high frequency noise from data.
    It has the advantage of preserving the original shape and
    features of the signal better than other types of filtering
    approaches, such as moving averages techniques.
    Parameters
    ----------
    y : array_like, shape (N,)
        the values of the time history of the signal.
    window_size : int
        the length of the window. Must be an odd integer number.
    order : int
        the order of the polynomial used in the filtering.
        Must be less then `window_size` - 1.
    deriv: int
        the order of the derivative to compute (default = 0 means only smoothing)
    Returns
    -------
    ys : ndarray, shape (N)
        the smoothed signal (or it's n-th derivative).
    Notes
    -----
    The Savitzky-Golay is a type of low-pass filter, particularly
    suited for smoothing noisy data. The main idea behind this
    approach is to make for each point a least-square fit with a
    polynomial of high order over a odd-sized window centered at
    the point.
    Examples
    --------
    t = np.linspace(-4, 4, 500)
    y = np.exp( -t**2 ) + np.random.normal(0, 0.05, t.shape)
    ysg = savitzky_golay(y, window_size=31, order=4)
    import matplotlib.pyplot as plt
    plt.plot(t, y, label='Noisy signal')
    plt.plot(t, np.exp(-t**2), 'k', lw=1.5, label='Original signal')
    plt.plot(t, ysg, 'r', label='Filtered signal')
    plt.legend()
    plt.show()
    References
    ----------
    .. [1] A. Savitzky, M. J. E. Golay, Smoothing and Differentiation of
       Data by Simplified Least Squares Procedures. Analytical
       Chemistry, 1964, 36 (8), pp 1627-1639.
    .. [2] Numerical Recipes 3rd Edition: The Art of Scientific Computing
       W.H. Press, S.A. Teukolsky, W.T. Vetterling, B.P. Flannery
       Cambridge University Press ISBN-13: 9780521880688
    """
    import numpy as np
    from math import factorial

    try:
        window_size = np.abs(np.int(window_size))
        order = np.abs(np.int(order))
    except ValueError, msg:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order+1)
    half_window = (window_size -1) // 2
    # precompute coefficients
    b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
    m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs( y[1:half_window+1][::-1] - y[0] )
    lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    return np.convolve( m[::-1], y, mode='valid')


#interpolate to n_samples
def interpolate_demo(demo, dt, runtime=1.0):
    time_steps = int(runtime/dt)
    ydes = np.zeros((demo.shape[0], time_steps))
    x = np.linspace(0, demo.shape[1]*dt, demo.shape[1], endpoint=False)
    #x = np.arange(0.0, demo.shape[1]*dt, dt)
    for d in range(demo.shape[0]):
        path_gen = scipy.interpolate.interp1d(x, demo[d])
        for t in xrange(time_steps):
            ydes[d, t] = path_gen(t * dt)
    return ydes


#CVXOPT nao gosta de inteiros-> passar para floats, tambem nao gosta de valores grandes na matriz h
def mult_opt_mp_from_path(demo, dt=0.001, n_bfs=100, q_max=50, dq_max=500.0, ddq_max=1000.0, jerk_max=10000, scale_acc=True, PW=None, VW=None):

    demo_time = demo.shape[1] * dt
    tau = demo_time  # (/1.0) change demo to time 1.0

    #demo = interpolate_demo(demo, dt=dt)

    t = np.arange(0, demo.shape[1]*dt, dt)
    T = t.shape[0]

    #centers = np.linspace(0, 1, n_bfs).reshape((1, n_bfs))
    centers = np.linspace(0, demo_time, n_bfs).reshape((1, n_bfs))
    hs = np.zeros_like(centers)
    hs[0, :-1] = (np.diff(centers)*0.65)**2
    hs[0, -1] = hs[0, -2]
    hs = 1./hs

    demo_d = np.zeros_like(demo)
    demo_dd = np.zeros_like(demo)

    #low pass differentiation
    for i in xrange(demo_dd.shape[0]):
        demo_dd[i, :] = savitzky_golay(demo[i, :], 201, 5, deriv=2, rate=1)/(dt**2)

    psi = np.exp(-hs.T * (t - centers.T)**2)
    PSI = psi.dot(psi.T)
    D = (psi.dot(demo_dd.T)).reshape(n_bfs, demo.shape[0])

    #0.5*x.T.dot(P.dot(x)) + q.T.dot(x)
    #G.dot(x) <= h
    #A.dot(x) = b

    P = cvx.matrix(PSI*2)
    Q = cvx.matrix((-2*D))
    A = cvx.matrix(np.vstack((np.arange(T,0,-1).dot(psi.T), np.ones(T).dot(psi.T))))
    n=n_bfs
    G = cvx.matrix(0.0, (n, n))
    G[::n+1] = 0.0
    h = cvx.matrix(100.0, (n, 1))

    int_term = (0.5*np.tri(T, T, -1)+np.tri(T, T, -1).dot(np.tri(T, T, -1)))

    #create process pool
    p = Pool(processes=N_CPU)

    #@profile
    def solve_new_target(P0, PF, V0, VF, tf, runtime):
        tau = tf / demo_time
        dt2 = tau*dt
        #T = int(tf/dt)
        bs = cvx.matrix(np.hstack(( (PF-P0-T*V0*dt2).T, ((VF-V0)).T )) ).T

        #solvers.options['show_progress'] = False
        #solvers.options['maxiters'] = 1

        #new psi for scaled time
        #t_gen = gen_canonical_time(tau, dt)
        """
        ts_t = t * tau # scale times
        centers_t = tau * centers
        hs_t = hs / tau**2

        #ts = [next(t_gen) for i in xrange(T)]
        psi = np.exp(-hs_t.T * (ts_t - centers_t.T)**2)
        PSI = psi.dot(psi.T)"""

        D = (psi.dot(demo_dd.T)).reshape(n_bfs, demo.shape[0])

        G1 = cvx.matrix(np.identity(T).dot(psi.T))#TODO: retirar identity, nao faz nada
        G12 = -1.0*G1
        G2 = dt2*cvx.matrix(np.tri(T, T, -1).dot(psi.T)) #+V0
        G22 = -1.0*G2
        G3 = cvx.matrix(dt2**2*int_term.dot(psi.T))# +P0+dtV0
        G32 = -1.0*G3

        #No Jerk################################
        #G = cvx.matrix([G1, G12, G2, G22, G3, G32])
        #h = cvx.matrix(1.0, (6*T, P0.shape[1]))
        ########################################

        #Jerk Minimization######################
        #buf = np.zeros_like(psi.T)
        #buf[0:-1, :] = (psi[:, 1:]-psi[:, 0:-1]).T
        buf = (psi[:, 1:]-psi[:, 0:-1]).T
        G4 = cvx.matrix(buf)
        G41 = -1.0 * G4
        G = cvx.matrix([G1, G12, G2, G22, G3, G32, G4, G41])
        h = cvx.matrix(1.0, (8*T-2, P0.shape[1]))
        #########################################

        h[:2*T, :] = ddq_max
        for i in xrange(P0.shape[1]):
            h[2*T:3*T, i] = (dq_max-V0)[0, i]#(vmax-V0)/dt
            h[3*T:4*T, i] = (dq_max+V0)[0, i]#(vmax-V0)/dt

            h[4*T:5*T, i] = (q_max-P0[0, i] - dt2*V0[0, i]*np.arange(1, T+1).reshape(T, 1))  #(pmax -P0 -dt*V0)/dt**2
            h[5*T:6*T, i] = (q_max+P0[0, i] + dt2*V0[0, i]*np.arange(1, T+1).reshape(T, 1)) #(pmax -P0 -dt*V0)/dt**2"""

        #jerk#####################################
        h[6*T:7*T-1, :] = dt*jerk_max
        h[7*T-1:8*T-2, :] = dt*jerk_max
        ##########################################

        P = cvx.matrix(PSI*2)

        var = 0.5 * np.ones(T)
        var[:T-2] += np.arange(T-2, 0, -1)  # replicar ultima linha de 0.5 tri + tri.tri
        A = cvx.matrix(np.vstack(( (dt2**2 * var).dot(psi.T), dt2*np.ones(T).dot(psi.T) )) )

        int2_way = int_term[499,:]
        int_way = np.ones(T)
        int_way[499:] = 0.0

        if PW is not None:
            A_wp = cvx.matrix(np.vstack(( (dt2**2 * var).dot(psi.T), dt2*np.ones(T).dot(psi.T), (dt2**2 * int2_way).dot(psi.T), dt2*int_way.dot(psi.T) )) )
            bbs_way = cvx.matrix(np.hstack(((PF-P0-T*V0*dt2).T, ((VF-V0)).T, (PW-P0-499*V0*dt2).T, ((VW-V0)).T ) ) ).T
            bs = bbs_way
            A = A_wp

        if scale_acc:  # scale accelerations (default)
            #test
            #solvers.qp(P, Q[:, 0], G, h[:, 0], A, bs[:, 0])['x']
            par_opt = ParOpt((P, Q/(tau**2), G, h, A, bs))
        else:
            par_opt = ParOpt((P, Q, G, h, A, bs))

        #multiprocessing
        #pmap = p.map(par_opt, range(P0.shape[1]))
        #ws = np.hstack(pmap).T

        #serial (for profiling)
        smap = map(par_opt, range(P0.shape[1]))
        ws = np.hstack(smap).T

        #New Psi for trajectory:
        #ts_t = np.arange(0, runtime, dt)
        #centers_t = tau * centers
        #hs_t = hs / tau**2
        #psi = np.exp(-hs_t.T * (ts_t - centers_t.T)**2)

        ydd = ws.dot(psi)
        fyd = ws.dot(psi[:, 1:]-psi[:, 0:-1])

        if int(runtime/dt2) > ydd.shape[1]:
            out = np.zeros((ydd.shape[0], int(runtime/dt2) - ydd.shape[1]))
            ydd = np.concatenate((ydd, out), axis=1)
        dy = np.zeros_like(ydd)
        y = np.zeros_like(ydd)
        #integracao simpletica
        v = V0.copy()/dt2
        x1 = P0.copy()
        for i in xrange(ydd.shape[1]):
            dv = ydd[:, i]
            dy[:, i] = dt2 * v
            y[:, i] = x1
            x1 += (dt2 ** 2) * v
            v += dv

        return y, dy, ydd
    return solve_new_target

class ParOpt:
    def __init__(self, state):
        self._state = state

    def __call__(self, ind):
        P, Q, G, h, A, bs = self._state
        return solvers.qp(P, Q[:, ind], G, h[:, ind], A, bs[:, ind])['x']

def work(ParOpt):
    ParOpt.call()


def gen_canonical_time(tau, dt):
    z = 0.0
    while True:
        yield z
        z += dt / tau


def gen_traj((y, dy, ddy), dt):  # generator to retrieve the generated trajectory like the implementation of dmps
    end_y = y[0, -1]
    for i in itertools.count():
        if i < y.shape[1]:
            yield y[0, i], dy[0, i], ddy[0, i]
        else:
            end_y += dt * dy[0, -1]
            yield end_y, dy[0, -1], ddy[0, -1]


#to test noise free and disturbance rejection by the trajectory execution
def acceleration_based_controller((g, dg, ddg), dt, noise=(0.0, 0.0, 0.0)):
    #dv = ((ay * (by * (g - y) + tau * dg - v)) + (ddg * tau ** 2) + eta * f) / tau
    #dy = v / tau
    #ddy = dv / tau

    #v += dv * dt
    #y += dy * dt

    vy, vdy, vddy = np.zeros_like(g), np.zeros_like(g), np.zeros_like(g)

    Gain=1.0
    ay, by = Gain*15.0, Gain*15.0/4.0
    y = g[0]
    v = dg[0]

    step_func = np.zeros_like(g)
    step_func[int(0.1*g.shape[0]):int(0.3*g.shape[0])] = 100.0

    for i in xrange(vy.shape[0]):
        r1, r2, r3 = noise * (np.random.random_sample(3)-0.5)

        #dv = (ay * (by * ((g[i]+r1) - y) + (dg[i]+r2) - v)) + (ddg[i] + r3)
        dv = (ay * (by * (g[i] - (y+r1)) + dg[i] - (v+r2))) + ddg[i] + step_func[i]
        dy = v
        ddy = dv

        v += dv * dt
        y += dy * dt
        vy[i], vdy[i], vddy[i] = y, dy, ddy
    return vy, vdy, vddy


#CVXOPT nao gosta de inteiros-> passar para floats, tambem nao gosta de valores grandes na matriz h
def opt_from_demo_unconstrained(demo, dt=0.001, n_bfs=100, scale_acc=True, PW=None, VW=None):
    demo_time = demo.shape[1] * dt

    t = np.arange(0, demo.shape[1]*dt, dt)
    T = t.shape[0]

    #centers = np.linspace(0, 1, n_bfs).reshape((1, n_bfs))
    centers = np.linspace(0, demo_time, n_bfs).reshape((1, n_bfs))
    hs = np.zeros_like(centers)
    hs[0, :-1] = (np.diff(centers)*0.65)**2
    hs[0, -1] = hs[0, -2]
    hs = 1./hs

    demo_dd = np.zeros_like(demo)

    #low pass differentiation
    n_dof = demo_dd.shape[0]
    for i in xrange(n_dof):
        demo_dd[i, :] = savitzky_golay(demo[i, :], 501, 2, deriv=2, rate=1)/(dt**2)

    psi = np.exp(-hs.T * (t - centers.T)**2)
    PSI = psi.dot(psi.T)
    D = (psi.dot(demo_dd.T)).reshape(n_bfs, demo.shape[0])

    #0.5*x.T.dot(P.dot(x)) + q.T.dot(x)
    #G.dot(x) <= h
    #A.dot(x) = b

    P = cvx.matrix(PSI*2)
    Q = cvx.matrix((-2*D))
    A = cvx.matrix(np.vstack((np.arange(T,0,-1).dot(psi.T), np.ones(T).dot(psi.T))))
    n=n_bfs
    G = cvx.matrix(0.0, (n, n))
    G[::n+1] = 0.0
    h = cvx.matrix(100.0, (n, 1))

    int_term = (0.5*np.tri(T, T, -1)+np.tri(T, T, -1).dot(np.tri(T, T, -1)))

    #process pool
    p = Pool(processes=N_CPU)

    #solvers.options['show_progress'] = False
    #solvers.options['maxiters'] = 3


    #@profile
    def solve_new_target(P0, PF, V0, VF, tf, runtime, PW=None, VW=None):
        tau = tf / demo_time
        dt2 = tau*dt
        #T = int(tf/dt)
        bs = cvx.matrix(np.hstack(( (PF-P0-T*V0*dt2).T, ((VF-V0)).T )) ).T

        #solvers.options['show_progress'] = False
        #solvers.options['maxiters'] = 5
        #D = (psi.dot(demo_dd.T)).reshape(n_bfs, demo.shape[0])

        '''
        G1 = cvx.matrix(np.identity(T).dot(psi.T))#TODO: retirar identity, nao faz nada
        G12 = -1.0*G1
        G2 = dt2*cvx.matrix(np.tri(T, T, -1).dot(psi.T)) #+V0
        G22 = -1.0*G2
        G3 = cvx.matrix(dt2**2*int_term.dot(psi.T))# +P0+dtV0
        G32 = -1.0*G3

        #No Jerk################################
        #G = cvx.matrix([G1, G12, G2, G22, G3, G32])
        #h = cvx.matrix(1.0, (6*T, P0.shape[1]))
        ########################################

        #Jerk Minimization######################
        #buf = np.zeros_like(psi.T)
        #buf[0:-1, :] = (psi[:, 1:]-psi[:, 0:-1]).T
        buf = (psi[:, 1:]-psi[:, 0:-1]).T
        G4 = cvx.matrix(buf)
        G41 = -1.0 * G4
        G = cvx.matrix([G1, G12, G2, G22, G3, G32, G4, G41])
        h = cvx.matrix(1.0, (8*T-2, P0.shape[1]))
        #########################################

        h[:2*T, :] = ddq_max
        for i in xrange(P0.shape[1]):
            h[2*T:3*T, i] = (dq_max-V0)[0, i]#(vmax-V0)/dt
            h[3*T:4*T, i] = (dq_max+V0)[0, i]#(vmax-V0)/dt

            h[4*T:5*T, i] = (q_max-P0[0, i] - dt2*V0[0, i]*np.arange(1, T+1).reshape(T, 1))  #(pmax -P0 -dt*V0)/dt**2
            h[5*T:6*T, i] = (q_max+P0[0, i] + dt2*V0[0, i]*np.arange(1, T+1).reshape(T, 1)) #(pmax -P0 -dt*V0)/dt**2"""

        #jerk#####################################
        h[6*T:7*T-1, :] = dt*jerk_max
        h[7*T-1:8*T-2, :] = dt*jerk_max
        ##########################################
        '''
        G = cvx.matrix(np.zeros((1, n_bfs)))
        h = cvx.matrix(np.ones((1, n_dof)))

        var = 0.5 * np.ones(T)
        var[:T-2] += np.arange(T-2, 0, -1)  # replicar ultima linha de 0.5 tri + tri.tri
        A = cvx.matrix(np.vstack(( (dt2**2 * var).dot(psi.T), dt2*np.ones(T).dot(psi.T) )) )

        int2_way = int_term[249, :]
        int_way = np.ones(T)
        int_way[249:] = 0.0

        if PW is not None:
            A_wp = cvx.matrix(np.vstack(( (dt2**2 * var).dot(psi.T), dt2*np.ones(T).dot(psi.T), (dt2**2 * int2_way).dot(psi.T), dt2*int_way.dot(psi.T) )) )
            #A_wp = cvx.matrix(np.vstack(( (dt2**2 * var).dot(psi.T), dt2*np.ones(T).dot(psi.T), (dt2**2 * int2_way).dot(psi.T), np.zeros_like(dt2*int_way.dot(psi.T)) )) )
            bbs_way = cvx.matrix(np.hstack(((PF-P0-T*V0*dt2).T, ((VF-V0)).T, (PW-P0-499*V0*dt2).T, ((VW-V0)).T ) ) ).T
            #bbs_way = cvx.matrix(np.hstack(((PF-P0-T*V0*dt2).T, ((VF-V0)).T, (PW-P0-499*V0*dt2).T, np.zeros_like((VW-V0)).T ) ) ).T
            bs = bbs_way
            A = A_wp

        if scale_acc:  # scale accelerations (default)
            #test
            #solvers.qp(P, Q[:, 0], G, h[:, 0], A, bs[:, 0])['x']
            par_opt = ParOpt((P, Q/(tau**2), G, h, A, bs))
        else:
            par_opt = ParOpt((P, Q, G, h, A, bs))

        #multiprocessing
        #pmap = p.map(par_opt, range(P0.shape[1]))
        #ws = np.hstack(pmap).T

        #serial (for profiling)
        smap = map(par_opt, range(P0.shape[1]))
        ws = np.hstack(smap).T

        ydd = ws.dot(psi)
        fyd = ws.dot(psi[:, 1:]-psi[:, 0:-1])

        if int(runtime/dt2) > ydd.shape[1]:
            out = np.zeros((ydd.shape[0], int(runtime/dt2) - ydd.shape[1]))
            ydd = np.concatenate((ydd, out), axis=1)
        dy = np.zeros_like(ydd)
        y = np.zeros_like(ydd)

        #integracao simpletica
        v = V0.copy()/dt2
        x1 = P0.copy()
        for i in xrange(ydd.shape[1]):
            dv = ydd[:, i]
            dy[:, i] = dt2 * v
            y[:, i] = x1
            x1 += (dt2 ** 2) * v
            v += dv

        return y, dy, ydd
    return solve_new_target


def opt_subsample(demo, dt=0.001, n_bfs=100, q_max=50, dq_max=500.0, ddq_max=1000.0, jerk_max=10000, scale_acc=True, PW=None, VW=None):

    demo_time = demo.shape[1] * dt
    tau = demo_time  # (/1.0) change demo to time 1.0

    #demo = interpolate_demo(demo, dt=dt)

    t = np.arange(0, demo.shape[1]*dt, dt)
    T = t.shape[0]

    #centers = np.linspace(0, 1, n_bfs).reshape((1, n_bfs))
    centers = np.linspace(0, demo_time, n_bfs).reshape((1, n_bfs))
    hs = np.zeros_like(centers)
    hs[0, :-1] = (np.diff(centers)*0.65)**2
    hs[0, -1] = hs[0, -2]
    hs = 1./hs

    demo_d = np.zeros_like(demo)
    demo_dd = np.zeros_like(demo)

    # if len(demo.shape) == 1:
    #     n_samples = len(demo)
    #     demo_d[:-1] = np.diff(demo) / dt
    #     demo_dd[:-1] = np.diff(demo_d) / dt
    #     demo_d[-1] = demo_d[-2]
    #     demo_dd[-2:] = demo_dd[-3]
    # else:
    #     n_samples = demo.shape[1]
    #     demo_d[:, :-1] = np.diff(demo) / dt
    #     demo_dd[:, :-1] = np.diff(demo_d) / dt
    #     demo_d[:, -1] = demo_d[:, -2]
    #     demo_dd[:, -2] = demo_dd[:, -3]
    #     demo_dd[:, -1] = demo_dd[:, -3]

    #low pass differentiation
    for i in xrange(demo_dd.shape[0]):
        demo_dd[i, :] = savitzky_golay(demo[i, :], 201, 5, deriv=2, rate=1)/(dt**2)

    psi = np.exp(-hs.T * (t - centers.T)**2)
    PSI = psi.dot(psi.T)
    D = (psi.dot(demo_dd.T)).reshape(n_bfs, demo.shape[0])

    #0.5*x.T.dot(P.dot(x)) + q.T.dot(x)
    #G.dot(x) <= h
    #A.dot(x) = b

    P = cvx.matrix(PSI*2)
    Q = cvx.matrix((-2*D))
    A = cvx.matrix(np.vstack((np.arange(T,0,-1).dot(psi.T), np.ones(T).dot(psi.T))))
    n=n_bfs
    G = cvx.matrix(0.0, (n, n))
    G[::n+1] = 0.0
    h = cvx.matrix(100.0, (n, 1))

    int_term = (0.5*np.tri(T, T, -1)+np.tri(T, T, -1).dot(np.tri(T, T, -1)))

    #create process pool
    p = Pool(processes=N_CPU)

    #@profile
    def solve_new_target(P0, PF, V0, VF, tf, runtime):
        tau = tf / demo_time
        dt2 = tau*dt
        #T = int(tf/dt)
        bs = cvx.matrix(np.hstack(( (PF-P0-T*V0*dt2).T, ((VF-V0)).T )) ).T

        #solvers.options['show_progress'] = False
        #solvers.options['maxiters'] = 1

        #new psi for scaled time
        #t_gen = gen_canonical_time(tau, dt)
        """
        ts_t = t * tau # scale times
        centers_t = tau * centers
        hs_t = hs / tau**2

        #ts = [next(t_gen) for i in xrange(T)]
        psi = np.exp(-hs_t.T * (ts_t - centers_t.T)**2)
        PSI = psi.dot(psi.T)"""

        D = (psi.dot(demo_dd.T)).reshape(n_bfs, demo.shape[0])

        G1 = cvx.matrix(np.identity(T).dot(psi.T))#TODO: retirar identity, nao faz nada
        G12 = -1.0*G1
        G2 = dt2*cvx.matrix(np.tri(T, T, -1).dot(psi.T)) #+V0
        G22 = -1.0*G2
        G3 = cvx.matrix(dt2**2*int_term.dot(psi.T))# +P0+dtV0
        G32 = -1.0*G3

        #No Jerk################################
        #G = cvx.matrix([G1, G12, G2, G22, G3, G32])
        #h = cvx.matrix(1.0, (6*T, P0.shape[1]))
        ########################################

        #Jerk Minimization######################
        #buf = np.zeros_like(psi.T)
        #buf[0:-1, :] = (psi[:, 1:]-psi[:, 0:-1]).T
        buf = (psi[:, 1:]-psi[:, 0:-1]).T
        G4 = cvx.matrix(buf)
        G41 = -1.0 * G4
        G = cvx.matrix([G1, G12, G2, G22, G3, G32, G4, G41])
        h = cvx.matrix(1.0, (8*T-2, P0.shape[1]))
        #########################################

        h[:2*T, :] = ddq_max
        for i in xrange(P0.shape[1]):
            h[2*T:3*T, i] = (dq_max-V0)[0, i]#(vmax-V0)/dt
            h[3*T:4*T, i] = (dq_max+V0)[0, i]#(vmax-V0)/dt

            h[4*T:5*T, i] = (q_max-P0[0, i] - dt2*V0[0, i]*np.arange(1, T+1).reshape(T, 1))  #(pmax -P0 -dt*V0)/dt**2
            h[5*T:6*T, i] = (q_max+P0[0, i] + dt2*V0[0, i]*np.arange(1, T+1).reshape(T, 1)) #(pmax -P0 -dt*V0)/dt**2"""

        #jerk#####################################
        h[6*T:7*T-1, :] = dt*jerk_max
        h[7*T-1:8*T-2, :] = dt*jerk_max
        ##########################################

        P = cvx.matrix(PSI*2)

        var = 0.5 * np.ones(T)
        var[:T-2] += np.arange(T-2, 0, -1)  # replicar ultima linha de 0.5 tri + tri.tri
        A = cvx.matrix(np.vstack(( (dt2**2 * var).dot(psi.T), dt2*np.ones(T).dot(psi.T) )) )

        int2_way = int_term[499,:]
        int_way = np.ones(T)
        int_way[499:] = 0.0

        if PW is not None:
            A_wp = cvx.matrix(np.vstack(( (dt2**2 * var).dot(psi.T), dt2*np.ones(T).dot(psi.T), (dt2**2 * int2_way).dot(psi.T), dt2*int_way.dot(psi.T) )) )
            bbs_way = cvx.matrix(np.hstack(((PF-P0-T*V0*dt2).T, ((VF-V0)).T, (PW-P0-499*V0*dt2).T, ((VW-V0)).T ) ) ).T
            bs = bbs_way
            A = A_wp

        if scale_acc:  # scale accelerations (default)
            #test
            #solvers.qp(P, Q[:, 0], G, h[:, 0], A, bs[:, 0])['x']
            par_opt = ParOpt((P, Q/(tau**2), G, h, A, bs))
        else:
            par_opt = ParOpt((P, Q, G, h, A, bs))

        #multiprocessing
        #pmap = p.map(par_opt, range(P0.shape[1]))
        #ws = np.hstack(pmap).T

        #serial (for profiling)
        smap = map(par_opt, range(P0.shape[1]))
        ws = np.hstack(smap).T

        #New Psi for trajectory:
        #ts_t = np.arange(0, runtime, dt)
        #centers_t = tau * centers
        #hs_t = hs / tau**2
        #psi = np.exp(-hs_t.T * (ts_t - centers_t.T)**2)

        ydd = ws.dot(psi)
        fyd = ws.dot(psi[:, 1:]-psi[:, 0:-1])

        if int(runtime/dt2) > ydd.shape[1]:
            out = np.zeros((ydd.shape[0], int(runtime/dt2) - ydd.shape[1]))
            ydd = np.concatenate((ydd, out), axis=1)
        dy = np.zeros_like(ydd)
        y = np.zeros_like(ydd)
        #integracao simpletica
        v = V0.copy()/dt2
        x1 = P0.copy()
        for i in xrange(ydd.shape[1]):
            dv = ydd[:, i]
            dy[:, i] = dt2 * v
            y[:, i] = x1
            x1 += (dt2 ** 2) * v
            v += dv

        return y, dy, ydd
    return solve_new_target

if __name__ == "__main__":
        n_basis = 60
        dt = 0.001
        movement_time = 1.0
        n_samples = int(movement_time/dt)
        t = np.arange(0,movement_time,dt)
        n_basis = 100
        demo = 2*t**2 + np.cos(t*4*np.pi)*(1-t) - 1;

        #add noise to demo
        demo += 0.001 * (2*np.random.random_sample(demo.shape)-1.0)

        V0 = -1.0768768403877083
        VF = 2.9945526656371957
        P0 = 0.0
        PF = 0.99700192104420382
        o_demo = demo.reshape(1, demo.shape[0])
        y0 = np.array([1.0*P0]).reshape(1,1)
        yf = np.array([1.0*PF]).reshape(1,1)
        dy0 = np.array([1.0*V0]).reshape(1,1)
        dyf = np.array([1.0*VF]).reshape(1,1)

        optf = mult_opt_mp_from_path(o_demo, dt=dt, n_bfs=n_basis)
        y_opt, dy_opt, ddy_opt = optf(y0, yf, dy0, dyf, tf=0.2, runtime=1.0)

        ddg=ddy_opt[0, :]
        dg=dy_opt[0, :]
        g=y_opt[0, :]

        acceleration_based_controller((g, dg, ddg) , 0.001, noise=0.0)

# #Multi DOF
# if __name__ == "__main__":
#     #artificial x5 trajectory
#     home =    np.array([0.0,   0.52, 0.82, -0.2,    0.0, 0.09])
#     prepare = np.array([-0.14, 0.52, 0.82, 0.1645,  0.0, -0.0353])
#     hit =     np.array([0.17,  0.47, 0.86, -0.0822, 0.0, 0.1058])
#     movement = np.vstack((home, home, home, prepare, hit, hit, hit))
#     times = np.array([0.0, 0.01, 0.02, 0.1, 0.5, 0.51, 0.52])
#
#     p0p = np.poly1d(np.polyfit(times,movement[:,0], 5))
#     p1p = np.poly1d(np.polyfit(times,movement[:,1], 5))
#     p2p = np.poly1d(np.polyfit(times,movement[:,2], 5))
#     p3p = np.poly1d(np.polyfit(times,movement[:,3], 5))
#     p5p = np.poly1d(np.polyfit(times,movement[:,5], 5))
#
#     p = np.polyfit(times,movement[:,0], 5)
#     pp = np.poly1d(p)
#     t = np.arange(0.0,0.5,0.001)
#
#     ts = np.linspace(0.0, 0.5, 500)
#     path = np.zeros((6, 500))
#     path[0, :] = p0p(ts)
#     path[1, :] = p1p(ts)
#     path[2, :] = p2p(ts)
#     path[3, :] = p3p(ts)
#     path[5, :] = p5p(ts)
#     y0 = path[:, 0].reshape(1, 6)
#     yf = path[:, -1].reshape(1, 6)
#     dy0 = np.zeros_like(y0)
#     dyf = np.zeros_like(y0)
#
#     optf = mult_opt_mp_from_path(path, dt=0.001, n_bfs=100)
#     y_opt, dy_opt, ddy_opt = optf(y0, yf, dy0, dyf, tf=0.2, runtime=1.0)
#
#     ddg=ddy_opt[0, :]
#     dg=dy_opt[0, :]
#     g=y_opt[0, :]



# if __name__ == "__main__":
#     n_basis = 100
#     dt = 0.001
#     movement_time = 1.0
#     n_samples = int(movement_time/dt)
#     t = np.arange(0,movement_time,dt)
#     n_basis = 100
#     demo = 2*t**2 + np.cos(t*4*np.pi)*(1-t) - 1
#
#     #add noise to demo
#     demo += 0.001 * (2*np.random.random_sample(demo.shape)-1.0)
#
#     V0 = -1.0768768403877083
#     VF = 2.9945526656371957
#     P0 = 0.0
#     PF = 0.99700192104420382
#     o_demo = demo.reshape(1, demo.shape[0])
#     y0 = np.array([1.0*P0]).reshape(1,1)
#     yf = np.array([1.0*PF]).reshape(1,1)
#     dy0 = np.array([1.0*V0]).reshape(1,1)
#     dyf = np.array([1.0*VF]).reshape(1,1)
#
#     optf = opt_from_demo_unconstrained(o_demo, dt=dt, n_bfs=60)
#     y_opt, dy_opt, ddy_opt = optf(y0, yf, dy0, dyf, tf=0.2, runtime=1.0)
#
#     ddg=ddy_opt[0, :]
#     dg=dy_opt[0, :]
#     g=y_opt[0, :]

