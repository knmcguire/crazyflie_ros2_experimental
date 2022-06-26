from setuptools import setup

package_name = 'crazyflie_ros2_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='knmcguire',
    maintainer_email='kimberly@bitcraze.io',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie_webots_driver = crazyflie_ros2_simulation.crazyflie_webots_driver:main'
        ],
    },
)
