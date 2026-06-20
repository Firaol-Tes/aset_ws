import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mm_llm_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Firaol-Tes',
    maintainer_email='firaalx@gmail.com',
    description='Closed-loop LLM task planner for the mobile manipulator (Phase 5)',
    license='BSD',
    entry_points={
        'console_scripts': [
            'planner_node = mm_llm_planner.planner_node:main',
            'send_task = mm_llm_planner.send_task:main',
        ],
    },
)
