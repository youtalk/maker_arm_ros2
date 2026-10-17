from glob import glob

from setuptools import find_packages, setup

package_name = 'maker_arm_mujoco'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', ['scripts/vcan_up.sh']),
    ],
    # mujoco is not a rosdep key: pip install mujoco==3.13.0 into the colcon venv (README.md).
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yutaka Kondo',
    maintainer_email='youtalk@users.noreply.github.com',
    description='MuJoCo plant for the MakerMods Maker Arm that answers on vcan0',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': ['plant = maker_arm_mujoco.plant_node:main']},
)
