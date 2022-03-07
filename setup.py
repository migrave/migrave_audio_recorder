# ! Do not manually invoke this setup.py. Use catkin instead


from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=["audio_recorder"],
    package_dir={"audio_recorder": "ros/src/audio_recorder"}
)

setup(**setup_args)
