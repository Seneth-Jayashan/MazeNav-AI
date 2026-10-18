# goal_interface: clicked-goal forwarding (goal_manager) and the measured
# 3-goal demonstration / benchmark (viva_demo).
from setuptools import setup

package_name = 'goal_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sjay',
    maintainer_email='sjay@sliit.lk',
    description='User clicked goal selection and automated VIVA demo for IE4060',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_manager = goal_interface.goal_manager:main',
            'viva_demo = goal_interface.viva_demo:main',
        ],
    },
)
