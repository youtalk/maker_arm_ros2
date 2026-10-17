from setuptools import find_packages, setup

package_name = 'maker_arm_demos'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yutaka Kondo',
    maintainer_email='youtalk@users.noreply.github.com',
    description='Demos of the MakerMods Maker Arm on MoveIt',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'random_goals = maker_arm_demos.random_goals:main',
        'servo_probe = maker_arm_demos.servo_probe:main',
    ]},
)
