from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_segmentation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@todo.todo',
    description='Camera segmentation -> LiDAR pointcloud projection pipeline',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'segmentation_node = amr_segmentation.segmentation_node:main',
            'pointcloud_painter_node = amr_segmentation.pointcloud_painter_node:main',
            'semantic_merger_node = amr_segmentation.semantic_merger_node:main',
        ],
    },
)
