from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['tape_recorder'],
        scripts=['scripts/tape.py'],
        package_dir={'': 'scripts'}
)

setup(**d)
