import os

from setuptools import setup

package_name = "crazyflie_ros2"
data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]


def package_files(_data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, _, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join("share", package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        _data_files.append((key, paths_dict[key]))

    return _data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=package_files(data_files, ["resource/", "launch/"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="knmcguire",
    maintainer_email="kimberly@bitcraze.io",
    description="Publishing Crazyflie Logging Variable",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "crazyflie_publisher = crazyflie_ros2.crazyflie_publisher:main"
        ]
    },
)
