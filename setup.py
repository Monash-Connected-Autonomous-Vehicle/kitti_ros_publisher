from setuptools import setup

package_name = 'kitti_ros_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcav',
    maintainer_email='senman_q@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kitti_ros_publisher = kitti_ros_publisher.kitti_ros_publisher:main',
            'kitti_ros_subscriber = kitti_ros_publisher.kitti_ros_subscriber:main'
        ],
    },
)
