import os
from glob import glob
from setuptools import setup

package_name = 'crazyflie_ros2_simulation'
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files(data_files, ['resource/', 'launch/', 'worlds/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='knmcguire',
    maintainer_email='kimberly@bitcraze.io',
    description='Files for Webots description Crazyflie',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie_webots_driver = crazyflie_ros2_simulation.crazyflie_webots_driver:main'
        ],
    },
)
