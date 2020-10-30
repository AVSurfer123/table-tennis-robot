from __future__ import division
import numpy as np
# "cimport" is used to import special compile-time information
# about the numpy module (this is stored in a file numpy.pxd which is
# currently part of the Cython distribution).
cimport cython
cimport numpy as np
# We now need to fix a datatype for our arrays. I've used the variable
# DTYPE for this, which is assigned to the usual NumPy runtime
# type info object.
DTYPE = np.float
# "ctypedef" assigns a corresponding compile-time type to DTYPE_t. For
# every type in the numpy module there's a corresponding compile-time
# type with a _t-suffix.
#ctypedef np.int_t DTYPE_t
ctypedef np.float_t DTYPE_t
# "def" can type its arguments but not have a return type. The type of the
# arguments for a "def" function is checked at run-time when entering the
# function.
#
# The arrays f, g and h is typed as "np.ndarray" instances. The only effect
# this has is to a) insert checks that the function arguments really are
# NumPy arrays, and b) make some attribute access like f.shape[0] much
# more efficient. (In this example this doesn't matter though.)


cdef np.ndarray R_COEF_TABLE = np.array([0.73, 0.73, -0.92])  # restitution coefficient of table (modificar tambem no no de trajectoria)
cdef float GRAV_ACC = -9.8

cdef float CW_DRAG_COEF = 0.47
cdef float P_AIR_DENS = 1.293  # [Kg/m^3]
cdef float A_SECTION = 0.0013
cdef float BALL_MASS = 0.0027

@cython.boundscheck(False) # turn of bounds-checking for entire function
@cython.wraparound(False)
def f_fun(double dt, np.ndarray x_pre_modified, np.ndarray x_post_unchanged,np.ndarray F_modified=None):
    cdef double den
    cdef np.ndarray val
    cdef float K
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
        val = np.sum(x_post_unchanged[0:3]**2) + (x_post_unchanged[3:6]**2)
        #den = np.linalg.norm(x_post[3:6]) # calculated previously
        if den != 0.0:
            K = -((CW_DRAG_COEF * P_AIR_DENS * A_SECTION) / (2 * BALL_MASS))
            val *= (K/den)
        else:
            val = np.zeros(3)
        np.fill_diagonal(F_modified[6:9, 3:6], val)  # d/d (dx) (-K*sqrt(dx^2+dy^2+dz^2)*[dx,dy,dz]
