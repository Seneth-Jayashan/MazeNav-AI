# maze_dqn: DQN training (train_dqn) and the ROS node that drives the robot
# with a trained network (dqn_navigation_node). models/ (weights, logs) is installed with it.
from glob import glob
from setuptools import setup

package_name = 'maze_dqn'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sjay',
    maintainer_email='sjay@sliit.lk',
    description='Deep Q-Learning navigation for TurtleBot3 Burger',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_dqn = maze_dqn.train_dqn:main',
            'dqn_navigation_node = maze_dqn.dqn_navigation_node:main',
        ],
    },
)
