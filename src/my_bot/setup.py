import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pure_pursuit_obstacle = my_bot.pure_pursuit_obstacle:main',
            'astar_obstacle = my_bot.astar_obstacle:main',
            'astar_obstacle3 = my_bot.astar_obstacle3:main',
            'astar_obstacle_cam = my_bot.astar_obstacle_cam:main',
        ],
    },
)

