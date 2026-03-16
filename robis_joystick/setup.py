from setuptools import setup

# import os
# from glob import glob

package_name = "robis_joystick"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Opcional: incluir arquivos de configuração, launch files, etc.
        # (os.path.join('share', package_name, 'launch'),
        #   glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seu_nome",
    maintainer_email="seu_email@exemplo.com",
    description="Controle de robô diferencial via joystick com pass-through",
    license="MIT",
    entry_points={
        "console_scripts": [
            "joystick_control = robis_joystick.joystick_control:main",
            "trajectory_controller = robis_joystick.trajectory_controller:main",
            "lemniscata_publisher = robis_joystick.lemniscata_publisher:main",
            "quadrado_publisher = robis_joystick.quadrado_publisher:main",
            "filtered_controller = robis_joystick.filtered_controller:main",
            "obstacle_turtle = robis_joystick.obstacle_turtle:main",
            "circle_publisher = robis_joystick.circle_publisher:main",
            "position_publisher = robis_joystick.position_publisher:main",
        ],
    },
)
