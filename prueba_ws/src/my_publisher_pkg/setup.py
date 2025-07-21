import os
import glob
from setuptools import find_packages, setup

package_name = 'my_publisher_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='aaa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = my_publisher_pkg.publisher_node:main',
            'pi_controller = my_publisher_pkg.pi_controller_node:main',
            'square_wave_publisher = my_publisher_pkg.square_wave_node:main',
            'zigzag_wave_publisher = my_publisher_pkg.zigzag_wave_node:main',
            'prbs_publisher = my_publisher_pkg.prbs_node:main',
        ],
    },
)

