from setuptools import find_packages, setup

package_name = 'terrain_obstacle_mapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ajoglek@tamu.edu',
    description='Terrain obstacle occupancy grid from depth pointcloud and segmentation mask',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'terrain_obstacle_grid_node = terrain_obstacle_mapper.terrain_obstacle_grid_node:main',
        ],
    },
)
