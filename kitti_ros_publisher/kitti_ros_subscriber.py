from pyexpat import XML_PARAM_ENTITY_PARSING_NEVER
from pyexpat.errors import XML_ERROR_INCOMPLETE_PE
from tokenize import String
#from uuid import RFC_4122
import numpy as np
from scipy import integrate
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
from nav_msgs.msg import Odometry
#from kitti_ros_subscriber.oxts_parser import *
import os
import shutil

class KittiRosSubscriber(Node):

    def __init__(self):
        super(KittiRosSubscriber, self).__init__('the_subscriber')

        #### Path Variables  
        # This needs to be fixed - most likely will with a launch file configuration
        self.cwd = "/home/mcav/liam_ws/localisation/localisation_ws/src/kitti_ros_publisher"
        self.dataDir = "/home/mcav/DATASETS"
        self.left_Id = "img02"
        self.right_Id = "img03"
        self.imu_Id = "oxts"
        self.lidar_Id = "velodyne_points"
        self.date = datetime.today().strftime('%Y-%m-%d')
        self.drive_number = "drive-nnnn-sync"
        self.directory_Id = os.path.join(self.dataDir, self.date)

        # Extra Variables
        self.lidar_topic = '/carla/ego_vehicle/lidar' #'/velodyne_points'
        self.point_stamped_topic = '/imu_pose'
        self.imu_topic = '/imu_output'
        self.odom_topic = '/carla/ego_vehicle/odom'
        self.imu_callback_counter = 0
        self.lidar_callback_counter = 0
        self.odom_callback_counter = 0
        self.last_rotMat = 0
        self.x_acc = np.empty(1)
        self.y_acc = np.empty(1)
        self.z_acc = np.empty(1)

        #### Calibration files
        self.cam_to_cam = "calib-cam-to-cam.txt"
        self.velo_to_cam = "calib-velo-to-cam.txt"
        self.imu_to_velo = "calib-imu-to-velo.txt"

        self.create_directory()

        #### Subscriptions
        self.create_subscription(PCL2,self.lidar_topic,self.lidar_callback, 10)

        #### Pose Estimation - Please choose one topic to subscribe to ####

        # 1) Uncomment this for Pose estimation from PointStamped topic
        #self.create_subscription(PointStamped,self.point_stamped_topic,self.imu_pstamped_callback, 10)

        
        # 2) Uncomment this for Pos estimation of IMU topic
        # self.create_subscription(Imu,self.imu_topic,self.imu_callback, 10)

        # 3) Uncomment this for pose from Odometry Topic
        self.create_subscription(Odometry,self.odom_topic,self.odom_callback, 10)

    
    def create_directory(self):
        """Initial Method to create all directories and files in KITTI format
        Args:
            Na
        Returns:
            Na
        """
        if os.path.isdir(self.directory_Id):
            # Add new drive
            list_of_dirs = [ name for name in os.listdir(self.directory_Id) if os.path.isdir(os.path.join(self.directory_Id, name)) ]
            # -4 gets last drive excluding calib files
            drive = (list_of_dirs[0].split("-")[4])
            self.drive_number = "drive-" + str(int(drive)+1).zfill(4) +"-sync"
        else:
            # First Drive of Day
            self.drive_number = "drive-0001-sync"
        
        self.directory_template('Lidar')
        self.directory_template('Imu')
        self.directory_template('Img02')
        self.directory_template('Img03')

        # Create Calibration Files
        cal1Path = os.path.join(self.directory_Id, self.cam_to_cam)
        cal2Path = os.path.join(self.directory_Id, self.velo_to_cam)
        cal3Path = os.path.join(self.directory_Id, self.imu_to_velo)
        f = open(cal1Path, 'w')
        f = open(cal2Path, 'w')
        f = open(cal3Path, 'w')

    def directory_template(self, sensor):
        """Method to create directories and some files in KITTI format.
        Args:
            String: Sensor this collecting data and needs directory setup
        Returns:
            Na
        """
        
        needDataFormat = 0
        if sensor == 'Lidar': sensorId = self.lidar_Id
        elif sensor == 'Imu': 
            sensorId = self.imu_Id
            needDataFormat = 1
        elif sensor == 'Img02': sensorId = "image_02" #self.left_Id
        elif sensor == 'Img03': sensorId = "image_03" #self.right_Id

        sensorPath = os.path.join(self.directory_Id, self.date + '-' + self.drive_number, sensorId)
        dataPath = os.path.join(sensorPath,'data')
        timestampsPath = os.path.join(sensorPath, 'timestamps.txt')
        if not os.path.exists(dataPath):
            os.makedirs(dataPath)
            f = open(timestampsPath, 'w')
            if needDataFormat: 
                shutil.copyfile(os.path.join(self.cwd,'dataformat/dataformat.txt'), os.path.join(sensorPath,'dataformat.txt'))


    #### Callbacks
    def lidar_callback(self, msg):
        timestamps_file = os.path.join(self.directory_Id,self.date + '-' + self.drive_number,self.lidar_Id,'timestamps.txt')
        self.add_timestamp(timestamps_file, datetime.now())
        new_txt_file = str(self.lidar_callback_counter).zfill(10) + '.bin'
        lidarFile = os.path.join(self.directory_Id,self.date + '-' + self.drive_number,self.lidar_Id,'data',new_txt_file)
        self.write_PCL2_to_bin(msg, lidarFile)

        self.lidar_callback_counter += 1

    def imu_pstamped_callback(self, msg):
        timestamps_file = os.path.join(self.directory_Id,self.date + '-' + self.drive_number,self.imu_Id,'timestamps.txt')
        self.add_timestamp(timestamps_file, datetime.now())
        new_txt_file = str(self.imu_callback_counter) .zfill(10)+ '.txt'
        dataFile = os.path.join(self.directory_Id,self.date + '-' + self.drive_number,self.imu_Id,'data',new_txt_file)
        self.write_imu_pstamped_to_txt(msg, dataFile)
        

    def imu_callback(self, msg):
        timestamps_file = os.path.join(self.directory_Id,self.date + '-' + self.drive_number,self.imu_Id,'timestamps.txt')
        self.add_timestamp(timestamps_file, datetime.now())
        new_txt_file = str(self.imu_callback_counter) .zfill(10)+ '.txt'
        dataFile = os.path.join(self.directory_Id,self.date + '-' + self.drive_number,self.imu_Id,'data',new_txt_file)
        self.write_imu_to_txt(msg, dataFile)
        
        self.imu_callback_counter += 1

    def odom_callback(self, msg):
        timestamps_file = os.path.join(self.directory_Id,self.date + '-' + self.drive_number,self.imu_Id,'timestamps.txt')
        self.add_timestamp(timestamps_file, datetime.now())
        new_txt_file = str(self.imu_callback_counter) .zfill(10)+ '.txt'
        dataFile = os.path.join(self.directory_Id,self.date + '-' + self.drive_number,self.imu_Id,'data',new_txt_file)
        self.write_odom_to_txt(msg, dataFile)


    def write_PCL2_to_bin(self, msg, file_name):
        """Method to convert Lidar data in PCL2 message to binary format and write to file.
        Args:
            ROS Msg: pcl2 lidar data.
            String: File name to write to.
        Returns:
            Na
        """
        f = open(file_name, 'w')
        cloud_points = list(point_cloud2.read_points(msg, skip_nans=True))
        x = [i[0] for i in cloud_points]
        y = [i[1] for i in cloud_points]
        z = [i[2] for i in cloud_points]
        intensity = [i[3] for i in cloud_points]
        
        arr = np.zeros(len(x) + len(y) + len(z) + len(intensity), dtype=np.float32)
        arr[::4] = x
        arr[1::4] = y
        arr[2::4] = z
        arr[3::4] = intensity
        print("Writing Lidar message of size: " + str(len(arr)) + " to binaryfile")
        arr.astype('float32').tofile(file_name)

    def write_imu_pstamped_to_txt(msg, file_name):
        """Method to convert Imu PointStamped data in Imu message to txt format and write to file.
        Args:
            ROS Msg: imu PointStamped data.
            String: File name to write to.
            Currently no estimation of orientation on this topic
        Returns:
            Na
        """
        f = open(file_name, 'w')
        dist_x = msg.point.x
        dist_y = msg.point.position.y
        dist_z = msg.point.position.z
        roll = 0
        pitch = 0
        yaw = 0
        data_string = str(dist_x) + " " + str(dist_y) + " " + str(dist_z) + " " + str(roll) + " " + str(pitch) + " " + str(yaw)
        f.write(data_string)
        f.close()
        
        
    def write_imu_to_txt(self, msg, file_name):
        """Method to convert Imu data in Imu message to txt format and write to file.
        Args:
            ROS Msg: imu data.
            String: File name to write to
        Returns:
            Na
        """
        # Currently defined to work with geometry_msgs/msg/PoseStamped
        f = open(file_name, 'w')
        self.x_acc = np.append(self.x_acc, msg.linear_acceleration.x)
        self.y_acc = np.append(self.y_acc, msg.linear_acceleration.y)
        self.z_acc = np.append(self.z_acc, msg.linear_acceleration.z)
        print(self.x_acc)

        x_ori = msg.orientation.x
        y_ori = msg.orientation.y
        z_ori = msg.orientation.z
        w_ori = msg.orientation.w

        roll, pitch, yaw = self.euler_from_quaternion(x_ori, y_ori, z_ori, w_ori)
        rot_mat = self.euler_to_rotMat(yaw, pitch, roll)

        # Data Structure Currently is a Euler Pose: "X Y Z"
        if self.imu_callback_counter == 0:
            # Set initial Pose to zeros
            self.last_rotMat = rot_mat
            data_string = "0 0 0 0 0 0"
        else:
            vel_x = integrate.cumtrapz(self.x_acc)
            dist_x = integrate.cumtrapz(vel_x)
            vel_y = integrate.cumtrapz(self.y_acc)
            dist_y = integrate.cumtrapz(vel_y)
            vel_z = integrate.cumtrapz(self.z_acc)
            dist_z = integrate.cumtrapz(vel_z)

            # We start with R01 and then we get R0x: we want R(X-1)(X)
            #R02 = R01*R12
            #R01_1*R02 = R12 
            curr_rot_mat = np.matmul(np.linalg.inv(self.last_rotMat),rot_mat)
            self.last_rotMat = curr_rot_mat
            roll, pitch, yaw = self.rot2eul(curr_rot_mat)
            data_string = str(dist_x[-1]) + " " + str(dist_y[-1]) + " " + str(dist_z[-1]) + " " + str(roll) + " " + str(pitch) + " " + str(yaw)
        f.write(data_string)
        f.close()

    def write_odom_to_txt(msg, file_name):
        f = open(file_name, 'w')
        dist_x = msg.pose.pose.position.x
        dist_y = msg.pose.pose.position.y
        dist_z = msg.pose.pose.position.z
        roll = msg.pose.pose.orientation.x
        pitch = msg.pose.pose.orientation.y
        yaw = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        data_string = str(dist_x) + " " + str(dist_y) + " " + str(dist_z) + " " + str(roll) + " " + str(pitch) + " " + str(yaw) + " " + str(w)
        f.write(data_string)
        f.close()

    def add_timestamp(self, file, time):
        f = open(file, 'a')
        f.write(str(time)+"\n")
        f.close()

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def euler_to_rotMat(self, yaw, pitch, roll):
        Rz_yaw = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [          0,            0, 1]])
        Ry_pitch = np.array([
            [ np.cos(pitch), 0, np.sin(pitch)],
            [             0, 1,             0],
            [-np.sin(pitch), 0, np.cos(pitch)]])
        Rx_roll = np.array([
            [1,            0,             0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll),  np.cos(roll)]])
        rotMat = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
        return rotMat

    def rot2eul(self, R):
        beta = -np.arcsin(R[2,0])
        alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
        gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
        return alpha, beta, gamma


def main(args=None):
    rclpy.init(args=args)
    kitti_ros_subscriber = KittiRosSubscriber()

    try:
        rclpy.spin(kitti_ros_subscriber)
    except KeyboardInterrupt:
        kitti_ros_subscriber.get_logger().debug("Keyboard interrupt")

    # destroy node explicity
    kitti_ros_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()