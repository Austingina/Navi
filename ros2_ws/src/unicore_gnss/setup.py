from glob import glob

from setuptools import find_packages, setup

package_name = "unicore_gnss"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="nav_test",
    maintainer_email="dev@example.com",
    description="Unicore N4 BESTNAVA serial GNSS driver for ROS 2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "unicore_gnss_node = unicore_gnss.driver_node:main",
        ],
    },
)
