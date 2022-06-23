from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['aurmr_web_interface'],
    package_dir={'': 'grasp_generation'}
)

setup(**d)