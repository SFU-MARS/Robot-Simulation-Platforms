from setuptools import find_packages, setup

package_name = 'turtlebot3_controls'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/hj_launch_world.launch.py']),
        ('share/' + package_name + '/worlds',
            ['hj_reach_target.world']),
        ('share/' + package_name + '/params',
            [
                'params/turtlebot3_burger_bridge.yaml',
                'params/turtlebot3_waffle_bridge.yaml',
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rafaelg',
    maintainer_email='rafaelg@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controls_keyboard=turtlebot3_controls.script.controls_keyboard:main',
        ],
    },
)
