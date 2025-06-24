#ifndef M300_E_PROTOCOL_H
#define M300_E_PROTOCOL_H

#include"define.h"

#define WIDTH 360
#define HEIGHT 150
#define FL7901_HIST_BIN_NUM 288


#define UDP_PIXEL_NUM 120
#pragma pack(push, 1)

	struct CmdHeader
	{
		unsigned short sign; //CN:与硬件约定的标志位						EN:Flags consistent with hardware
		unsigned short ver;
		unsigned short msgid;//消息ID
		unsigned short cmd;	 //CN:命令										EN：command
		unsigned int len;	//CN:命令长度									EN:command length
	};
struct Point3D
    {
        double x;
        double y;
        double z;
    };
    struct onePoi
    {
        uint16_t row_pos;
        uint16_t col_pos;
        float distance;		   // 单位 cm
        uint32_t reflectivity; // 反射率
        Point3D pt3d;		   // 真实坐标值
        uint32_t peak0_pos;	   // 峰 1 的位置
        uint32_t peak0_val;	   // 峰 1 的高度
        uint32_t peak1_pos;	   // 峰 2 的位置
        uint32_t peak1_val;	   // 峰 2 的高度
        uint16_t envSumXi;
        uint32_t nbVal;
        uint32_t centDis;	// 质心距离
        uint32_t riseDis;	// 上升沿距离
        uint32_t firPeak;	// FIR 峰高
        uint64_t timestamp; // Unit:us
    };
    typedef onePoi oneFramePoi[HEIGHT][WIDTH];
    typedef struct tag_soc_head
    {						   // size 6
        uint16_t head;		   // 固定包头，0x5A,0xA5
        uint8_t version_major; // 协议主版本，当前为 1
        uint8_t data_type;	   // 数据类型，//点云数据 0x01
        uint16_t reserved;	   // 保留字段
    } soc_head_t;
    // 数据协议
    typedef struct tag_soc_protocol_data
    {							// size 40
        uint8_t version_minor;	// 协议子版本，当前为 0
        uint16_t data_sequence; // 数据序列，可用于统计接收数据是否丢包
        uint16_t frame_cnt;		// 帧序号
        uint8_t time_type;		// 时间戳类型
        uint16_t col;			// 当前数据包的起始 col
        uint16_t row;			// 当前数据包的起始 row
        uint16_t dot_num;		// 当前数据包的 data 字段包含的点的数量
        uint8_t type;			// 点云数据类型，（Distance/XYZ/(Distance+XYZ) ）+ 反射率 + 标签，默认 XYZ
        uint8_t echo_num;		// 点云数据的回波个数，默认双回波，最大支持回波个数是 4 个回波。分别是最强、次强、最近、最远。
        uint8_t frame_flag;		// 0 frame start; 1 normal piexl 2:frame end;
        uint8_t reserved[11];	// 保留字段
        uint8_t crc8;			// 主协议头+子协议头的检验码，使用 crc-8 算法
        uint32_t crc32;			// timestamp+data 段校验码，使用 CRC-32 算法
        uint64_t timestamp;		// Unit:us
        uint8_t data[0];		// 数据信息
    } soc_protocol_data_t;
    typedef struct tag_soc_dis_pt_data
    {
        uint16_t distance; // 当前回波的距离值
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t reflect; // 当前回波的反射率
        uint8_t label;	 // 保留，跟点云测量到的实际物体有关，如雨雾、灰尘等
    } soc_dis_pt_data_t;
    typedef struct fs_lidar_imu
    {
        uint8_t crc8_1;
        uint8_t crc8_2;
        uint8_t version;
        uint16_t len;
        uint8_t time_type;
        uint64_t timestamp; // Unit:ns
        uint8_t reserved[2];
        float gyro_x; // Unit:rad/s
        float gyro_y; // Unit:rad/s
        float gyro_z; // Unit:rad/s
        float acc_x;  // Unit:g
        float acc_y;  // Unit:g
        float acc_z;  // Unit:g
    } fs_lidar_imu_t;


typedef struct
{
	uint32_t second;
	uint32_t nano_second;
}TIME_ST;

struct KeepAlive {
		TIME_ST world_clock;//毫秒级时间戳
		TIME_ST arrive_time;//包数据主机到雷达的时间(内部使用)
		TIME_ST delay_time;//延迟时间(内部使用)
	};
#pragma pack(pop)
typedef void(*LidarCloudPointCallback) (uint32_t handle, const uint8_t dev_type, onePoi*data, void *client_data);
typedef void(*LidarImuDataCallback)(uint32_t handle, const uint8_t dev_type, fs_lidar_imu_t* data, void* client_data);
typedef void(*LidarLogDataCallback)(uint32_t handle, const uint8_t dev_type, char* data, int len);



#endif
