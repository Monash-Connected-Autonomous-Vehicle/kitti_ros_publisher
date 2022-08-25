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
        self.lidar_topic = '/velodyne_points'
        self.imu_topic = '/imu_pose'
        self.imu_callback_counter = 0
        self.lidar_callback_counter = 0

        #### Calibration files
        self.cam_to_cam = "calib-cam-to-cam.txt"
        self.velo_to_cam = "calib-velo-to-cam.txt"
        self.imu_to_velo = "calib-imu-to-velo.txt"

        self.create_directory()

        #### Subscriptions
        self.create_subscription(PCL2,self.lidar_topic,self.lidar_callback, 10)
        self.create_subscription(PointStamped,self.imu_topic,self.imu_callback, 10)

        ## NEED TO IMPLEMENT: Imu -> Pose
        #self.create_subscription(Imu,self.imu_topic,self.imu_callback, 10)

    
    def create_directory(self):
        """Initial Method to create all directories and files in KITTI format
        Args:
            Na
        Returns:
            Na
        """
        if os.path.isdir(self.directory_Id):
            # Add new drive
            list_of_dirs = os.listdir(self.directory_Id)
            # -4 gets last drive excluding calib files
            drive = list_of_dirs[0].split("-")[4]
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

    def imu_callback(self, msg):
        timestamps_file = os.path.join(self.directory_Id,self.date + '-' + self.drive_number,self.imu_Id,'timestamps.txt')
        self.add_timestamp(timestamps_file, datetime.now())
        new_txt_file = str(self.imu_callback_counter) .zfill(10)+ '.txt'
        dataFile = os.path.join(self.directory_Id,self.date + '-' + self.drive_number,self.imu_Id,'data',new_txt_file)
        self.write_imu_to_txt(msg, dataFile)
        
        self.imu_callback_counter += 1

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
        # Data Structure Currently is a Euler Pose: "X Y Z"
        if self.imu_callback_counter == 0:
            # Set initial Pose to zeros
            data_string = "0 0 0 0 0 0"
        else:
            X = msg.point.x
            Y = msg.point.y
            Z = msg.point.z
            data_string = str(X) + " " + str(Y) + " " + str(Z) + " 0 0 0"
        f.write(data_string)
        f.close()

    def add_timestamp(self, file, time):
        f = open(file, 'a')
        f.write(str(time)+"\n")
        f.close()

    def euler_from_quaternion(x, y, z, w):
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