from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['uni_pal_pylib'],
    package_dir={'': 'utils'},
)

setup(
    **d,
    data_files=[
        ('share/' + "uni_pal_pylib", ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + "uni_pal_pylib"]),
    ],
)