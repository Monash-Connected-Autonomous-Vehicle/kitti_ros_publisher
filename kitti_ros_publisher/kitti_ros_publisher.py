import rclpy
from rclpy.node import Node

import numpy as np
import glob 

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2 as PCL2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2


class KittiRosPublisher(Node):

    def __init__(self):
        super(KittiRosPublisher, self).__init__('point_cloud_to_pcl2')
        self.plc2_pub = self.create_publisher(PCL2, 'pcl2conversion', 10)
        # self.image_pub = self.create_publisher(_, _, _)  # TODO
        # self.gps_pub = self.create_publisher(_, _, _)  # TODO
        # self.imu_pub = self.create_publisher(_, _, _)  # TODO
        # TODO: create publishers for other useful information

        timer_period = 0.5
        self.spinner = self.create_timer(timer_period, self.spin)

        # Paths
        data_path = 'data/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync'

        self.velodyne_file_paths = glob.glob(data_path + '/velodyne_points/data/*.bin')
        self.velodyne_timestamps_file = open(data_path + '/velodyne_points/timestamps.txt', 'r')
        self.velodyne_timestamps = self.timestamp_extract(self.velodyne_timestamps_file)

        self.leftImg_file_paths = glob.glob(data_path + '/image_02/data/*.png')
        self.leftImg_timestamps_file = open(data_path + '/image_02/timestamps.txt', 'r')
        self.leftImg_timestamps = self.timestamp_extract(self.leftImg_timestamps_file)
        
        self.rightImg_file_paths = glob.glob(data_path + '/image_03/data/*.png')
        self.rightImg_timestamps_fule = open(data_path + '/image_03/timestamps.txt', 'r')
        self.rightImg_timestamps = self.timestamp_extract(self.rightImg_timestamps_file)
        
        self.oxts_file_paths = glob.glob(data_path + '/oxts/data/*.txt')
        self.oxts_timestamps_fule = open(data_path + '/oxts/timestamps.txt', 'r')
        self.oxts_timestamps = self.timestamp_extract(self.oxts_timestamps_file)


    ### Callback functions
    def spin(self):
        """Main callback"""
        self.plc2_pubber()
        self.image_pubber()
        self.oxts_pubber()


    def plc2_pubber(self):
        """Callback to PLC2 publisher"""
        if self.velodyne_file_paths:
            msg = self.convert_bin_to_PCL2(self.velodyne_file_paths.pop())
            self.plc2_pub.publish(msg)
        else:
            self.get_logger().info("no more velodyne points to publish...")


    def image_pubber(self):
        """Callback to image publisher"""
        pass # TODO


    def oxts_pubber(self):
        """Callback to oxts publisher"""
        pass # TODO


    ### Miscellaneous functions
    def timestamp_extract(self, txt_file):
        """Converts timestamp file to list"""
        timestamp_list =[]

        for timestamp in txt_file.readlines():
            timestamp_list.append([timestamp])

        return timestamp_list


    def convert_bin_to_PCL2(self, velodyne_file_path):
        """Method to convert Lidar data in binary format to PCL2 message"""
        
        cloud = np.fromfile(velodyne_file_path, np.float32)
        cloud = cloud.reshape((-1, 4))
        # x, y, z, r = cloud[::4], cloud[1::4], cloud[2::4], cloud[3::4]

        header = Header()
        header.frame_id = 'velodyne'
        header.stamp = self.velodyne_timestamps.pop()
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        pcl2_msg = point_cloud2.create_cloud(header, fields, cloud)

        self.get_logger().info(f"Publishing, first point: {cloud[0:5]}")

        return pcl2_msg


def main(args=None):
    rclpy.init(args=args)

    kitti_ros_publisher = KittiRosPublisher()

    try:
        rclpy.spin(kitti_ros_publisher)
    except KeyboardInterrupt:
        kitti_ros_publisher.get_logger().debug("Keyboard interrupt")

    # destroy node explicity
    kitti_ros_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
