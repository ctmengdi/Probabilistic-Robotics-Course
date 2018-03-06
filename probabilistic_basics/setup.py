## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=['lib/probabilistic_lib/functions.py'],
    packages=['probabilistic_lib'],
    package_dir={'': 'lib'},
    requires=['geometry_msgs',
              'nav_msgs',
              'rospy',
              'sensor_msgs',
              'std_msgs',
              'visualization_msgs'])

setup(**setup_args)
