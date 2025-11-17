from setuptools import find_packages, setup

package_name = "ros_consumer"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="labvisio",
    maintainer_email="labvisio@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "publisher = ros_consumer.publisher:main",
            "consumer = ros_consumer.consumer:main",
            "talker = ros_consumer.utils.examples.talker:main",
            "listener = ros_consumer.utils.examples.listener:main",
            "simulator = ros_consumer.utils.simulator:main",
            "plot_simulator = ros_consumer.utils.plot_simulator:main",
        ],
    },
)
