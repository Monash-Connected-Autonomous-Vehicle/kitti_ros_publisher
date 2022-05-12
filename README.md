# KITTI Publisher

Repo to allow fo easy use of a basic KITTI point cloud publisher for LiDAR. Other sensorpublishers to be added in future.


## Getting Started - PCL2 Publisher

### Prerequisites
* create a ros2 workspace by following [these](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) instructions
* clone this repository into the src directory in your ros2 workspace

### Installation

1. Download synced+rectified data from [here](http://www.cvlibs.net/datasets/kitti/raw_data.php) or to download the data directly, press [this](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0048/2011_09_26_drive_0048_sync.zip) download button
2. After the download is complete, navigate to ```velodyne_points/data``` then copy-and-paste the binary files to ```data/velodyne_points``` in the project folders
3. navigate to the root folder of your workspace
4. build the project
	```sh
    colcon build
    . install/setup.bash
    ```
    
5. in a new terminal, call the publisher
	```sh
    . install/setup.bash
    ros2 run kitti_publisher pcl2_pub
    ```
    
## Contact
Amir Toosi - amir.ghalb@gmail.com

Ben Edwards - bedw0004@student.monash.edu
