# S300  driver

---

## SDK

windows/linux SDK and Demo programs for  LDS-S300-E lidar

### HOW TO BUILD AND USE

***EXE file explain:***

>PointCloudAndImu:    Output point cloud data of each frame of lidar and real-time imu data
>CommandControl:      Output lidar supported commands and return values

### LINUX

***Prerequisite: g++ and gcc must be installed***

    cmake CMakeList.txt
    make
    ./${EXE file}            EXE file{PointCloudAndImu,CommandControl}

### WINDOWS

use cmake_gui  build ,open project and compile with visual stdioxx,then enerate a solution

---

## ROS

BLUESEA ROS driver is specially designed to connect to the lidar products produced by our company. The driver can run on operating systems with ROS installed, and mainly supports ubuntu series operating systems (14.04LTS-20.04LTS). The hardware platforms that have been tested to run the ROS driver include: Intel x86 mainstream CPU platform, and some ARM64 hardware platforms (such as NVIDIA, Rockchip, Raspberry Pi, etc., which may need to update the cp210x driver).

***Please ensure that the path does not contain Chinese characters, otherwise the compilation will fail!***

### Get  Build  Run

    git clone https://github.com/BlueSeaLidar/S300.git 
    cd S300/S300-ROS/
    catkin_make
    source devel/setup.sh
    roslaunch pacecat_s300 LDS-S300_E.launch

## Driver launch launch file

>frame_id : Name of the flag coordinate system
>topic_pointcloud : pointcloud  topic print
>output_pointcloud: pointcloud  topic print enable
>topic_custommsg  : custom msg topic print
>output_custommsg : custom msg topic print enable
>topic_imu        : imu topic
>output_imu       :imu topic enable
>lidar_ip         :lidar ip
>lidar_port       :lidar port
>local_port       :listen port