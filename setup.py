from setuptools import setup

package_name = "simulated_robots"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jblumenkamp",
    maintainer_email="jb2270@cam.ac.uk",
    description="A collection of different simulated robots",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot = simulated_robots.simple_turtlebot:main",
            "robomaster = simulated_robots.simple_robomaster:main",
            "minicar = simulated_robots.simple_minicar:main",
        ],
    },
)
