#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <math.h>
#include <thread>
#include <mutex>
#include <pacecat_s300/CustomMsg.h>
#include <pacecat_s300/CustomPoint.h>
#include <queue>
#include "../sdk/sdk.h"

typedef struct
{
  std::string frame_id;

  ros::Publisher pub_pointcloud;
  std::string topic_pointcloud;
  bool output_pointcloud;

  ros::Publisher pub_custommsg;
  std::string topic_custommsg;
  bool output_custommsg;

  ros::Publisher pub_imu;
  std::string topic_imu;
  bool output_imu;

  std::string lidar_ip;
  int lidar_port;
  int local_port;
} ArgData;

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, onePoi *data, void *client_data)
{
  if (data == nullptr)
  {
    return;
  }
   //printf("point cloud handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
   //handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
 
    onePoi *p_point_data = (onePoi *)data;
    ArgData *argdata = (ArgData *)client_data;
    if (argdata->output_pointcloud)
    {
      sensor_msgs::PointCloud msg;
      int N = WIDTH*HEIGHT;
      msg.header.stamp.sec = data[0].timestamp / 1000000;
      msg.header.stamp.nsec = (data[0].timestamp % 1000000)*1000;
      msg.header.frame_id = argdata->frame_id;
      msg.points.resize(N);
      msg.channels.resize(1);
      msg.channels[0].name = "intensities";
      msg.channels[0].values.resize(N);

      for (size_t i = 0; i < N; i++)
      {
        msg.points[i].x = p_point_data[i].pt3d.x;
        msg.points[i].y = p_point_data[i].pt3d.y;
        msg.points[i].z = p_point_data[i].pt3d.z;
        msg.channels[0].values[i] = p_point_data[i].reflectivity;
      }
      sensor_msgs::PointCloud2 laserCloudMsg;
      convertPointCloudToPointCloud2(msg, laserCloudMsg);
      argdata->pub_pointcloud.publish(laserCloudMsg);
    }
    if (argdata->output_custommsg)
    {
      pacecat_s300::CustomMsg msg;
      int N =  WIDTH*HEIGHT;
      msg.point_num = N;
      msg.lidar_id = 0;
      msg.header.frame_id = argdata->frame_id;
      msg.header.stamp.sec = data[0].timestamp / 1000000;
      msg.header.stamp.nsec = (data[0].timestamp % 1000000)*1000;

      msg.header.seq++;
      msg.rsvd = {0, 0, 0};
      for (size_t i = 0; i < N; i++)
      {
        pacecat_s300::CustomPoint point;
        point.x = p_point_data[i].pt3d.x;
        point.y = p_point_data[i].pt3d.y;
        point.z = p_point_data[i].pt3d.z;
        point.reflectivity = p_point_data[i].reflectivity;
        point.offset_time =  (data[0].timestamp- data[i].timestamp)%1000000;
        msg.points.push_back(point);
      }
      // printf("%d %d\n", N, msg.points.size());
      argdata->pub_custommsg.publish(msg);
    }
  
 
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type, fs_lidar_imu_t *data, void *client_data)
{
  if (data == nullptr)
  {
    return;
  }
  // printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
  //        handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
    ArgData *argdata = (ArgData *)client_data;
    if (argdata->output_imu)
    {
      fs_lidar_imu_t *p_imu_data = (fs_lidar_imu_t *)data;
      sensor_msgs::Imu imu;
      imu.angular_velocity.x = p_imu_data->gyro_x;
      imu.angular_velocity.y = p_imu_data->gyro_y;
      imu.angular_velocity.z = p_imu_data->gyro_z;

      imu.linear_acceleration.x = p_imu_data->acc_x;
      imu.linear_acceleration.y = p_imu_data->acc_y;
      imu.linear_acceleration.z = p_imu_data->acc_z;

      uint64_t nanosec = p_imu_data->timestamp;
      imu.header.frame_id = argdata->frame_id;
      // std::cout << p_imu_data->timestamp << std::endl;
      imu.header.stamp.sec = p_imu_data->timestamp /  1000000000;
      imu.header.stamp.nsec = p_imu_data->timestamp % 1000000000;
      argdata->pub_imu.publish(imu);
    }
  


}

void LogDataCallback(uint32_t handle, const uint8_t dev_type, char *data, int len)
{
  if (data == nullptr)
  {
    return;
  }
  printf("ID::%d print level:%d msg:%s\n", handle, dev_type, data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Lidar_M300");
  ros::NodeHandle nh("~");
  ArgData argdata;

  nh.param("frame_id", argdata.frame_id, std::string("map"));

  nh.param("topic_pointcloud", argdata.topic_pointcloud, std::string("pointcloud"));
  nh.param("output_pointcloud", argdata.output_pointcloud, true);

  nh.param("topic_custommsg", argdata.topic_custommsg, std::string("custommsg"));
  nh.param("output_custommsg", argdata.output_custommsg, true);

  nh.param("topic_imu", argdata.topic_imu, std::string("imu"));
  nh.param("output_imu", argdata.output_imu, true);

  nh.param("lidar_ip", argdata.lidar_ip, std::string("192.168.158.98"));

  nh.param("lidar_port", argdata.lidar_port, 6543);
  nh.param("local_port", argdata.local_port, 6668);

  if (argdata.output_pointcloud)
    argdata.pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>(argdata.topic_pointcloud, 10);
  if (argdata.output_custommsg)
    argdata.pub_custommsg = nh.advertise<pacecat_s300::CustomMsg>(argdata.topic_custommsg, 10);
  if (argdata.output_imu)
    argdata.pub_imu = nh.advertise<sensor_msgs::Imu>(argdata.topic_imu, 10);

  PaceCatLidarSDK::getInstance()->Init();
  int devID = PaceCatLidarSDK::getInstance()->AddLidar(argdata.lidar_ip, argdata.lidar_port, argdata.local_port);

  PaceCatLidarSDK::getInstance()->SetPointCloudCallback(devID, PointCloudCallback, &argdata);
  PaceCatLidarSDK::getInstance()->SetImuDataCallback(devID, ImuDataCallback, &argdata);
  PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);

  PaceCatLidarSDK::getInstance()->ConnectLidar(devID);

  while (ros::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
