from glob import glob
import os

from setuptools import setup

package_name = 'swarm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wenis',
    maintainer_email='marinathonda@gmail.com',
    description='Swarm tools and examples: swarm CLI (takeoff, formations, land), decentralised formation follower and GCS swarm monitor',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'swarm_cli = swarm_control.swarm_cli:main',
            'formation_follower = swarm_control.formation_follower:main',
            'swarm_monitor = swarm_control.swarm_monitor:main',
        ],
    },
)
