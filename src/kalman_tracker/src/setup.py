__author__ = 'carlos'
from distutils.core import setup
from Cython.Build import cythonize



setup(
    name = 'Cython Tracker Node',
    ext_modules = cythonize("cython_tracker.pyx"),
)