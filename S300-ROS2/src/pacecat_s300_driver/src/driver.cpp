#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include "pacecat_s300_inter/msg/custom_msg.hpp"
#include "pacecat_s300_inter/msg/custom_point.hpp"
#include "pacecat_s300_inter/srv/control.hpp"
#include "../sdk/pacecatlidarsdk.h"
#include "../sdk/global.h"
enum PointField
{
  INT8 = 1,
  UINT8,
  INT16,
  UINT16,
  INT32,
  UINT32,
  FLOAT32,
  FLOAT64
};
using namespace std::placeholders;
// typedef struct
// {
//   std::string frame_id;

//   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud;
//   std::string topic_pointcloud;
//   bool output_pointcloud;

//   rclcpp::Publisher<pacecat_s300_inter::msg::CustomMsg>::SharedPtr pub_custommsg;
//   std::string topic_custommsg;
//   bool output_custommsg;

//   rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
//   std::string topic_imu;
//   bool output_imu;
//   std::string adapter;
// } PubTopic;
typedef struct
{
  std::string frame_id;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud;
  std::string topic_pointcloud;
  bool output_pointcloud;

  rclcpp::Publisher<pacecat_s300_inter::msg::CustomMsg>::SharedPtr pub_custommsg;
  std::string topic_custommsg;
  bool output_custommsg;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
  std::string topic_imu;
  bool output_imu;

  std::string lidar_ip;
  int lidar_port;
  int local_port;
  std::string adapter;

} ArgData;

#ifdef ROS_VERSION_MAJOR

#endif
#include <chrono>
onePoi *p_point_data = NULL;
// using namespace std::chrono;
// time_point<steady_clock> last_time_;
// int count_;
void PointCloudCallback(uint32_t handle, const uint8_t dev_type, onePoi *data,uint16_t num, void *client_data)
{
  if (data == nullptr)
  {
    return;
  }
  if (p_point_data == NULL)
    p_point_data = new onePoi[num];

    memcpy(p_point_data, data, sizeof(onePoi) * num);
    ArgData *argdata = (ArgData *)client_data;
    if (argdata->output_pointcloud)
    {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.frame_id.assign(argdata->frame_id);
      cloud.height = 1;
      cloud.width = num;
      cloud.fields.resize(5);
      cloud.fields[0].offset = 0;
      cloud.fields[0].name = "x";
      cloud.fields[0].count = 1;
      cloud.fields[0].datatype = PointField::FLOAT32;

      cloud.fields[1].offset = 4;
      cloud.fields[1].name = "y";
      cloud.fields[1].count = 1;
      cloud.fields[1].datatype = PointField::FLOAT32;

      cloud.fields[2].offset = 8;
      cloud.fields[2].name = "z";
      cloud.fields[2].count = 1;
      cloud.fields[2].datatype = PointField::FLOAT32;

      cloud.fields[3].offset = 12;
      cloud.fields[3].name = "intensity";
      cloud.fields[3].count = 1;
      cloud.fields[3].datatype = PointField::UINT32;

      cloud.fields[4].offset = 16;
      cloud.fields[4].name = "timestamp";
      cloud.fields[4].count = 1;
      cloud.fields[4].datatype = PointField::FLOAT64;

      cloud.point_step = 24;
      cloud.row_step = cloud.width * cloud.point_step;
      cloud.data.resize(cloud.row_step * cloud.height);

      for (size_t i = 0; i < num; i++)
      {
        memcpy(&cloud.data[0] + i * 24, &p_point_data[i].pt3d.x, 4);
        memcpy(&cloud.data[0] + i * 24 + 4, &p_point_data[i].pt3d.y, 4);
        memcpy(&cloud.data[0] + i * 24 + 8, &p_point_data[i].pt3d.z, 4);
        memcpy(&cloud.data[0] + i * 24 + 12, &p_point_data[i].reflectivity, 4);
        memcpy(&cloud.data[0] + i * 24 + 16, &p_point_data[i].timestamp, 8);
      }
      cloud.header.stamp.sec = data->timestamp / 1000000;
      cloud.header.stamp.nanosec = data->timestamp % 1000000;


      // auto now = steady_clock::now();
      //   auto duration = duration_cast<std::chrono::milliseconds>(now - last_time_).count();
      //   last_time_ = now;
      //   count_++;

      //   // 每10次打印一次平均间隔（验证是否真的10Hz执行）
      //   if (count_ % 10 == 0) {
      //       printf("回调平均间隔: %ld ms, 理论10Hz应为100ms\n", duration);
      //   }

      argdata->pub_pointcloud->publish(cloud);
    }
    if (argdata->output_custommsg)
    {
      pacecat_s300_inter::msg::CustomMsg msg;
      uint16_t N = num;
      msg.point_num = N;
      msg.lidar_id = 0;
      msg.header.frame_id = argdata->frame_id;
      msg.timebase = data->timestamp;
      msg.header.stamp.sec = data->timestamp / 1000000;
      msg.header.stamp.nanosec = data->timestamp % 1000000;

      msg.rsvd = {0, 0, 0};
      for (size_t i = 0; i < N; i++)
      {
        pacecat_s300_inter::msg::CustomPoint point;
        point.x = p_point_data[i].pt3d.x;
        point.y = p_point_data[i].pt3d.y;
        point.z = p_point_data[i].pt3d.z;
        point.reflectivity = p_point_data[i].reflectivity;
        msg.points.push_back(point);
      }
      argdata->pub_custommsg->publish(msg);
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
      sensor_msgs::msg::Imu imu;
      imu.angular_velocity.x = p_imu_data->gyro_x;
      imu.angular_velocity.y = p_imu_data->gyro_y;
      imu.angular_velocity.z = p_imu_data->gyro_z;

      imu.linear_acceleration.x = p_imu_data->acc_x;
      imu.linear_acceleration.y = p_imu_data->acc_y;
      imu.linear_acceleration.z = p_imu_data->acc_z;

      uint64_t nanosec = data->timestamp;
      imu.header.frame_id = argdata->frame_id;

      imu.header.stamp.sec = nanosec / 1000000000;
      imu.header.stamp.nanosec = nanosec % 1000000000;
      argdata->pub_imu->publish(imu);
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

#define READ_PARAM(TYPE, NAME, VAR, INIT) \
  VAR = INIT;                             \
  declare_parameter<TYPE>(NAME, VAR);     \
  get_parameter(NAME, VAR);

class LidarNode : public rclcpp::Node
{
public:
  LidarNode()
      : Node("lidar_s300")
  {
    // 读取参数
    // PubTopic pubtopic;
    READ_PARAM(std::string, "frame_id", argdata.frame_id, "map");
    READ_PARAM(std::string, "topic_pointcloud", argdata.topic_pointcloud, "pointcloud");
    READ_PARAM(bool, "output_pointcloud", argdata.output_pointcloud, true);
    READ_PARAM(std::string, "topic_custommsg", argdata.topic_custommsg, "custommsg");
    READ_PARAM(bool, "output_custommsg", argdata.output_custommsg, true);
    READ_PARAM(std::string, "topic_imu", argdata.topic_imu, "imu");
    READ_PARAM(bool, "output_imu", argdata.output_imu, true);
    READ_PARAM(std::string, "adapter", argdata.adapter, "eth0");

    READ_PARAM(std::string, "lidar_ip", argdata.lidar_ip, "192.168.158.98");
    READ_PARAM(int, "lidar_port", argdata.lidar_port, 6543);
    READ_PARAM(int, "local_port", argdata.local_port, 6668);
    // 创建发布者
    if (argdata.output_pointcloud)
    {
      argdata.pub_pointcloud = create_publisher<sensor_msgs::msg::PointCloud2>(
          argdata.topic_pointcloud, 10);
    }
    if (argdata.output_custommsg)
    {
      argdata.pub_custommsg = create_publisher<pacecat_s300_inter::msg::CustomMsg>(
          argdata.topic_custommsg, 10);
    }
    if (argdata.output_imu)
    {
      argdata.pub_imu = create_publisher<sensor_msgs::msg::Imu>(
          argdata.topic_imu, 10);
    }

    // 初始化SDK并设置回调
    PaceCatLidarSDK::getInstance()->Init(argdata.adapter);
    devID = PaceCatLidarSDK::getInstance()->AddLidar(argdata.lidar_ip, argdata.lidar_port, argdata.local_port);

    PaceCatLidarSDK::getInstance()->SetPointCloudCallback(devID, PointCloudCallback, &argdata);
    PaceCatLidarSDK::getInstance()->SetImuDataCallback(devID, ImuDataCallback, &argdata);
    PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);
    PaceCatLidarSDK::getInstance()->ConnectLidar(devID);
    // 创建服务端
    service = this->create_service<pacecat_s300_inter::srv::Control>("scan_conntrol", std::bind(&LidarNode::control, this, _1, _2));
  }

private:
  rclcpp::Service<pacecat_s300_inter::srv::Control>::SharedPtr service;
  // ArgData argdata;
  ArgData argdata;
  int devID;
  void control(const pacecat_s300_inter::srv::Control::Request::SharedPtr req, const pacecat_s300_inter::srv::Control::Response::SharedPtr res)
  {
    // LidarAction action = LidarAction::NONE;

    // if (req->func == "start")
    // {
    //   action = LidarAction::START;
    // }
    // else if (req->func == "stop")
    // {
    //   action = LidarAction::STOP;
    // }
    // RCLCPP_INFO(this->get_logger(), "set lidar action: %s", req->func.c_str());
    // res->code = PaceCatLidarSDK::getInstance()->SetLidarAction(devID, action);
    // if (res->code <= 0)
    //   res->value = "faild";
    // else if (res->code == 1)
    //   res->value = "OK";
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
