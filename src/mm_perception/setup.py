from setuptools import find_packages, setup

package_name = 'mm_perception'

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
    maintainer='Firaol-Tes',
    maintainer_email='firaalx@gmail.com',
    description='HSV + YOLOv8 perception node for the mobile manipulator',
    license='BSD',
    entry_points={
        'console_scripts': [
            'perception_node = mm_perception.perception_node:main',
        ],
    },
)
