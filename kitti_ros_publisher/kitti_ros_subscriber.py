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

        #### Variables  
        # This needs to be fixed - most likely will with a launch file configuration
        self.cwd = "/home/mcav/liam_ws/localisation/localisation_ws/src/kitti_ros_publisher"
        self.dataDir = "/home/mcav/DATASETS"
        #self.cwd = os.getcwd()
        self.left_Id = "img02"
        self.right_Id = "img03"
        self.imu_Id = "oxts"
        self.lidar_Id = "velodyne_points"
        self.date = "yyyy-mm-dd"
        self.drive_number = "drive-nnnn-sync"
        self.directory_Id = os.path.join(self.dataDir, self.date)

        #### Calibration files
        self.cam_to_cam = "calib-cam-to-cam.txt"
        self.velo_to_cam = "calib-velo-to-cam.txt"
        self.imu_to_velo = "calib-imu-to-velo.txt"

        self.create_directory()

        #### Subscriptions
        self.create_subscription(PCL2,'/velodyne_points',self.lidar_callback)

        #### Data paths
        self.data_path = 'data/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync'
        self.velodyne_file_paths = sorted(glob.glob(self.data_path + '/velodyne_points/data/*.bin'))
        self.leftImg_file_paths = sorted(glob.glob(self.data_path + '/image_02/data/*.png'))
        self.rightImg_file_paths = sorted(glob.glob(self.data_path + '/image_03/data/*.png'))
        self.oxts_file_paths = sorted(glob.glob(self.data_path + '/oxts/data/*.txt'))

    
    #### Creating Directories and Text files in appropriate format
    def create_directory(self):
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
        # Might need to change these to 'x' - exclusive not overwriting file

    #### Directory template used for directory creation
    def directory_template(self, sensor):
        needDataFormat = 0

        if sensor == 'Lidar': sensorId = self.lidar_Id
        elif sensor == 'Imu': 
            sensorId = self.imu_Id
            needDataFormat = 1
        elif sensor == 'Img02': sensorId = self.left_Id
        elif sensor == 'Img03': sensorId = self.right_Id

        sensorPath = os.path.join(self.directory_Id, self.date + '-' + self.drive_number, sensorId)
        dataPath = os.path.join(sensorPath,'data')
        timestampsPath = os.path.join(sensorPath, 'timestamps.txt')
        if not os.path.exists(dataPath):
            os.makedirs(dataPath)
            f = open(timestampsPath, 'w')
            if needDataFormat: 
                shutil.copyfile(os.path.join(self.cwd,'dataformat/dataformat.txt'), os.path.join(sensorPath,'dataformat.txt'))


    #### Callbacks
    def lidar_callback(self):
        ###
        pass

    def imu_callback(self):
        ###
        pass

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