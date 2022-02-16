from setuptools import setup
import glob

package_name = 'kitti_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name +'/data/velodyne_points/', glob.glob('data/velodyne_points/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benca',
    maintainer_email='bedw0004@student.monash.edu',
    description='Repo to allow for easy use of a basic KITTI point cloud publisher for LiDAR. Other sensorpublishers to be added in future.',
    license='The MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pcl2_pub = kitti_publisher.pcl2_pub:main',
        ],
    },
)
