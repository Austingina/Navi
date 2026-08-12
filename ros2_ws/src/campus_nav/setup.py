from glob import glob

from setuptools import find_packages, setup

package_name = "campus_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name, glob("*.gpkg") + glob("*.osm.pbf")),
        (
            "share/" + package_name + "/config/behavior_trees",
            glob("config/behavior_trees/*.xml"),
        ),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="nav_test",
    maintainer_email="dev@example.com",
    description="Campus OSM route planner for ROS 2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "route_planner_node = campus_nav.route_planner_node:main",
            "route_plan_gui = campus_nav.route_plan_gui:main",
        ],
    },
)
