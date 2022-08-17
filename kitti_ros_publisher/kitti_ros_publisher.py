from tokenize import String
import numpy as np
import glob 
import rclpy
import time
import cv2 as cv
import PIL
import math
import os
from cv_bridge import CvBridge
from rclpy.node import Node
from datetime import datetime
from collections import namedtuple
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2 as PCL2, PointField, Image as Img, Imu
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs_py import point_cloud2
from kitti_ros_publisher.oxts_parser import *


class KittiRosPublisher(Node):

    def __init__(self):
        super(KittiRosPublisher, self).__init__('point_cloud_to_pcl2')
        self.plc2_pub  = self.create_publisher(PCL2, '/velodyne_points', 10)
        self.leftImg_pub = self.create_publisher(Img, '/img_left', 10)
        self.rightImg_pub = self.create_publisher(Img, '/img_right', 10)
        self.imuOut_pub = self.create_publisher(Imu, '/imu_output', 10)
        self.imuPose_pub = self.create_publisher(PointStamped, '/imu_pose', 10)

        timer_period = 0.5
        self.spinner = self.create_timer(timer_period, self.spin)

        # Data paths
        self.data_path = '/home/mcav/DATASETS/KITTI/2011_09_26/2011_09_26_drive_0048_sync'
        self.velodyne_file_paths = sorted(glob.glob(self.data_path + '/velodyne_points/data/*.bin'))
        self.leftImg_file_paths = sorted(glob.glob(self.data_path + '/image_02/data/*.png'))
        self.rightImg_file_paths = sorted(glob.glob(self.data_path + '/image_03/data/*.png'))
        self.oxts_file_paths = sorted(glob.glob(self.data_path + '/oxts/data/*.txt'))

        # Timestamp paths
        self.velodyne_timestamps_file = open(self.data_path + '/velodyne_points/timestamps.txt', 'r')
        self.velodyne_timestamps = self.timestamp_extract(self.velodyne_timestamps_file)
        self.leftImg_timestamps_file = open(self.data_path + '/image_02/timestamps.txt', 'r')
        self.leftImg_timestamps = self.timestamp_extract(self.leftImg_timestamps_file)  
        self.rightImg_timestamps_file = open(self.data_path + '/image_03/timestamps.txt', 'r')
        self.rightImg_timestamps = self.timestamp_extract(self.rightImg_timestamps_file)
        self.oxts_timestamps_file = open(self.data_path + '/oxts/timestamps.txt', 'r')
        self.oxts_timestamps = self.timestamp_extract(self.oxts_timestamps_file)

        # Other variables
        self.left_Id = "img02"
        self.right_Id = "img03"
        self.oxts_origin = None
        self.oxts_scale = None


    ### Callback functions
    def spin(self):
        """Main callback"""
        self.plc2_pubber()
        self.image_pubber()
        self.oxts_pubber()


    def plc2_pubber(self):
        """Callback to PLC2 publisher"""
        if self.velodyne_file_paths:
            msg = self.convert_bin_to_PCL2(self.velodyne_file_paths.pop(0))
            self.plc2_pub.publish(msg)
        else:
            self.velodyne_file_paths = sorted(glob.glob(self.data_path + '/velodyne_points/data/*.bin'))
            self.get_logger().info("no more velodyne points to publish...")


    def image_pubber(self):
        """Callback to image publisher"""
        if self.leftImg_file_paths:
            msg = self.cv2ImageToRosImage(self.leftImg_file_paths.pop(0), self.left_Id)
            self.leftImg_pub.publish(msg)
            
        else:
            self.leftImg_file_paths = sorted(glob.glob(self.data_path + '/image_02/data/*.png'))
            self.get_logger().info("no more left images to publish...")

        if self.rightImg_file_paths:
            msg = self.cv2ImageToRosImage(self.rightImg_file_paths.pop(0), self.right_Id)
            self.rightImg_pub.publish(msg)
            
        else:
            self.rightImg_file_paths = sorted(glob.glob(self.data_path + '/image_03/data/*.png'))
            self.get_logger().info("no more right images to publish...")


    def oxts_pubber(self):
        """Callback to oxts publisher"""
        if self.oxts_file_paths:
            imu_msg, gps_msg = self.oxts_to_imu(self.oxts_file_paths.pop())
            self.imuOut_pub.publish(imu_msg)
            self.imuPose_pub.publish(gps_msg)
        else:
            self.oxts_file_paths = sorted(glob.glob(self.data_path + '/oxts/data/*.txt'))
            self.get_logger().info("no more oxts data to publish...")


    ### Miscellaneous functions
    # TODO: Currently unused
    def timestamp_extract(self, timestamp_file_path:list) -> list:
        """Converts timestamp text file to list.

        Args:
            timestamp_file_path (list): .txt file of timestamps.

        Returns:
            list: list of timestamps.
        """

        timestamp_list=[]

        for timestamp in timestamp_file_path.readlines():
            timestamp_list.append([timestamp])

        return timestamp_list


    def convert_bin_to_PCL2(self, velodyne_file_path:list) -> list:
        """Method to convert Lidar data in binary format to PCL2 message.

        Args:
            velodyne_file_path (list): path to lidar data files.
        Returns:
            list: pcl2 lidar data.
        """
        
        cloud = np.fromfile(velodyne_file_path, np.float32)
        cloud = cloud.reshape((-1, 4))

        header = Header()
        header.frame_id = 'velodyne'
        header.stamp = self.get_clock().now().to_msg()
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        pcl2_msg = point_cloud2.create_cloud(header, fields, cloud)

        self.get_logger().info(f"Publishing, first point: {cloud[0:5]}")

        return pcl2_msg


    def cv2ImageToRosImage(self, imgPath:str, Frame_Id:str) -> Img:
        """Method to convert openCV images to ROS Image message.

        Args:
            imgPath (str): path to image files
            Frame_Id (str): frame id of images

        Returns:
            Img: sensor_msgs/Image message
        """

        cv2_img = cv.imread(imgPath)

        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(cv2_img, encoding="passthrough")
        image_message.header.frame_id = Frame_Id

        self.get_logger().info(f"Publishing, image: {Frame_Id}")

        return image_message     


    def oxts_to_imu(self, oxts_filename:str) -> Tuple[Imu, PointStamped]:
        """Extracts imu information from oxts files.

        Args:
            oxts_filename (str): path to oxts files

        Returns:
            Tuple[Imu, PointStamped]: IMU message containing rotational, angular velocity and linear acceleration data; PointStamped message containing GPS data.
        """

        oxts_tf, oxtf_vel, oxtf_acc, self.oxts_origin = load_oxts_packets_and_poses([oxts_filename], self.oxts_origin, self.oxts_scale)
        
        R = oxts_tf[0][:3,:3]  # IMU rotation
        T = oxts_tf[0][:,-1]  # GPS coordinates

        # Quaternion
        qw = math.sqrt(1.0 + R[0,0] + R[1,1] + R[2,2]) / 2
        qx = (R[2,1] - R[1,2]) / (4 * qw)
        qy = (R[0,2] - R[2,0]) / (4 * qw)
        qz = (R[1,0] - R[0,1]) / (4 * qw)

        # GPS message
        gps_msg = PointStamped()
        gps_msg.header.frame_id = 'base_link'
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.point.x = T[0]
        gps_msg.point.y = T[1]
        gps_msg.point.z = T[2]

        # IMU message
        imu_msg = Imu()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        imu_msg.angular_velocity.x = oxtf_vel[0]
        imu_msg.angular_velocity.y = oxtf_vel[1]
        imu_msg.angular_velocity.z = oxtf_vel[2]
        imu_msg.linear_acceleration.x = oxtf_acc[0]
        imu_msg.linear_acceleration.y = oxtf_acc[1]
        imu_msg.linear_acceleration.z = oxtf_acc[2]

        return imu_msg, gps_msg


class Calibration():

    def __init__(self, filepath):

        self.filepath = filepath
  
        calib_velo_to_cam = self.read_calib_file(filepath + "calib_velo_to_cam.txt")
        calib_cam_to_cam  = self.read_calib_file(filepath + "calib_cam_to_cam.txt")
        calib_imu_to_velo = self.read_calib_file(filepath + "calib_imu_to_velo.txt")

        # camera calibration
        self.K = calib_cam_to_cam["K_02"]
        
        # Projection matrix from rect camera coord to image2 coord 
        self.P      = calib_cam_to_cam["P_rect_02"].reshape(3, 4)

        # rect mat
        R_rect = calib_cam_to_cam["R_rect_02"].reshape(3, 3)
        self.R_rect = self.transform_from_rot_trans(R_rect, np.zeros(3))


        # Rotation and Translation matrix (velodyne)
        R = calib_velo_to_cam["R"].reshape(3, 3)
        T = calib_velo_to_cam["T"].reshape(3, 1)

        # Rigid transform from Velodyne coord to reference camera coord
        self.T_velo_cam = np.concatenate((R, T), axis = 1)
        self.T_velo_cam = np.vstack([self.T_velo_cam, [0, 0, 0, 1]])

        # Rotation and Translation matrix (velodyne)
        R = calib_imu_to_velo["R"].reshape(3, 3)
        T = calib_imu_to_velo["T"].reshape(3, 1)

        # Rigid transform IMU coord from Velodyne coord
        self.T_imu_velo = np.concatenate((R, T), axis = 1)
        self.T_imu_velo = np.vstack([self.T_imu_velo, [0, 0, 0, 1]])

    def read_calib_file(self, filepath):
        """ Read in a calibration file and parse into a dictionary.
        Ref: https://github.com/utiasSTARS/pykitti/blob/master/pykitti/utils.py
        """

        data = {}
        with open(filepath, "r") as f:
            for line in f.readlines():
                line = line.rstrip()
                if len(line) == 0:
                    continue
                key, value = line.split(":", 1)
                # The only non-float values in these files are dates, which
                # we don't care about anyway
                try:
                    data[key] = np.array([float(x) for x in value.split()])
                except ValueError:
                    pass

        return data
    
    def transform_from_rot_trans(self, R, t):
        """
        Transformation matrix from rotation matrix and translation vector.
        Parameters
        ----------
        R : np.array [3,3]
            Rotation matrix
        t : np.array [3]
            translation vector
        Returns
        -------
        matrix : np.array [4,4]
            Transformation matrix
        """
        R = R.reshape(3, 3)
        t = t.reshape(3, 1)
        return np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))


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
