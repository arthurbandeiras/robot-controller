from setuptools import setup

package_name = "robis_joystick"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seu_nome",
    maintainer_email="seu_email@exemplo.com",
    description="Controle de robô diferencial via joystick com pass-through",
    license="MIT",
    entry_points={
        "console_scripts": ["joystick_control = robis_joystick.joystick_control:main"],
    },
)
