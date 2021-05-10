from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['rqt_mypkg'],
    #add to src PYTHONPATH
    package_dir={'': 'src'},
    #make it standalone
    scripts=['scripts/rqt_mypkg']
)

setup(**d)