from setuptools import setup

package_name = 'c21_behaviors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'lidar_behavior = c21_behaviors.lidar_behavior:main',
            'vision_follow = c21_behaviors.vision_follow:main',
        ],
    },
)
