#ifndef S300_E_PROTOCOL_H
#define S300_E_PROTOCOL_H

#include "define.h"

#define WIDTH 360
#define HEIGHT 150
#define FL7901_HIST_BIN_NUM 288
#define HEARTPORT 6789
#define UDP_PIXEL_NUM 120

enum IMUACC
{
    ACC_2 = 0,
    ACC_4,
    ACC_8,
    ACC_16
};
enum IMUGYRO
{
    GYRO_15_625 = 0,
    GYRO_31_25,
    GYRO_62_5,
    GYRO_125,
    GYRO_250,
    GYRO_500,
    GYRO_1000,
    GYRO_2000
};

enum IMU_FILTERLEVEL
{
    SAMPLE_RATE_DIV2 = 0,
    SAMPLE_RATE_DIV4,
    SAMPLE_RATE_DIV5,
    SAMPLE_RATE_DIV8,
    SAMPLE_RATE_DIV10,
    SAMPLE_RATE_DIV16,
    SAMPLE_RATE_DIV20,
    SAMPLE_RATE_DIV40
};
enum IMU_SAMPLERATE
{
    FREQ_25HZ = 0,
    FREQ_50HZ,
    FREQ_100HZ,
    FREQ_200HZ
};

#pragma pack(push, 1)
typedef struct
{
    uint32_t second;
    uint32_t nano_second;
} TIME_ST;

struct CmdHeader
{
    unsigned short sign; // CN:与硬件约定的标志位						EN:Flags consistent with hardware
    unsigned short ver;
    unsigned short msgid; // 消息ID
    unsigned short cmd;   // CN:命令										EN：command
    unsigned int len;     // CN:命令长度									EN:command length
};
struct Point3D
{
    float x;
    float y;
    float z;
};
struct onePoi
{
    uint16_t row_pos;
    uint16_t col_pos;
    float distance;        // 单位 cm
    uint32_t reflectivity; // 反射率
    Point3D pt3d;          // 真实坐标值
    uint32_t peak0_pos;    // 峰 1 的位置
    uint32_t peak0_val;    // 峰 1 的高度
    uint32_t peak1_pos;    // 峰 2 的位置
    uint32_t peak1_val;    // 峰 2 的高度
    uint16_t envSumXi;
    uint32_t nbVal;
    uint32_t centDis;   // 质心距离
    uint32_t riseDis;   // 上升沿距离
    uint32_t firPeak;   // FIR 峰高
    uint64_t timestamp; // Unit:us
};
typedef onePoi oneFramePoi[HEIGHT][WIDTH];
typedef struct tag_soc_head
{                          // size 6
    uint16_t head;         // 固定包头，0x5A,0xA5
    uint8_t version_major; // 协议主版本，当前为 1
    uint8_t data_type;     // 数据类型，//点云数据 0x01
    uint16_t reserved;     // 保留字段
} soc_head_t;
// 数据协议
typedef struct tag_soc_protocol_data
{                           // size 40
    uint8_t version_minor;  // 协议子版本，当前为 0
    uint16_t data_sequence; // 数据序列，可用于统计接收数据是否丢包
    uint16_t frame_cnt;     // 帧序号
    uint8_t time_type;      // 时间戳类型
    uint16_t col;           // 当前数据包的起始 col
    uint16_t row;           // 当前数据包的起始 row
    uint16_t dot_num;       // 当前数据包的 data 字段包含的点的数量
    uint8_t type;           // 点云数据类型，（Distance/XYZ/(Distance+XYZ) ）+ 反射率 + 标签，默认 XYZ
    uint8_t echo_num;       // 点云数据的回波个数，默认双回波，最大支持回波个数是 4 个回波。分别是最强、次强、最近、最远。
    uint8_t frame_flag;     // 0 frame start; 1 normal piexl 2:frame end;
    uint8_t reserved[11];   // 保留字段
    uint8_t crc8;           // 主协议头+子协议头的检验码，使用 crc-8 算法
    uint32_t crc32;         // timestamp+data 段校验码，使用 CRC-32 算法
    uint64_t timestamp;     // Unit:us
    uint8_t data[0];        // 数据信息
} soc_protocol_data_t;
typedef struct tag_soc_dis_pt_data
{
    uint16_t distance; // 当前回波的距离值
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t reflect; // 当前回波的反射率
    uint8_t label;   // 保留，跟点云测量到的实际物体有关，如雨雾、灰尘等
} soc_dis_pt_data_t;
typedef struct
{
    uint16_t distance; // 当前回波的距离值
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t reflect; // 当前回波的反射率
} soc_dis_pt_data_t_v1;
typedef struct
{
    uint8_t crc8_header;
    uint8_t crc8_imu_dat;
    uint8_t version;
    uint16_t len;
    uint8_t time_type;
    int64_t timestamp;
    int16_t temperature; // Unit: 1/256 ℃
    uint8_t data[0];
} fs_lidar_imu_header_t;

typedef struct
{
    float gyro_x; // Unit:rad/s
    float gyro_y; // Unit:rad/s
    float gyro_z; // Unit:rad/s
    float acc_x;  // Unit:g
    float acc_y;  // Unit:g
    float acc_z;  // Unit:g
} fs_lidar_imu_v2;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
    float gyro_x; // Unit:rad/s
    float gyro_y; // Unit:rad/s
    float gyro_z; // Unit:rad/s
    float acc_x;  // Unit:g
    float acc_y;  // Unit:g
    float acc_z;  // Unit:g
} fs_lidar_imu_v3;

typedef struct
{
    int64_t timestamp;
    uint8_t version;
    float pitch;
    float roll;
    float yaw;
    float gyro_x; // Unit:rad/s
    float gyro_y; // Unit:rad/s
    float gyro_z; // Unit:rad/s
    float acc_x;  // Unit:g
    float acc_y;  // Unit:g
    float acc_z;  // Unit:g

} fs_lidar_imu_t;

struct DEV_CFG_ST
{
    uint16_t cfg_ver; /* 参数结构体版本号 */

    char dev_type[16];   /* LDS-S00 */
    char dev_sn[32];     /* 设备 SN 号 */
    uint32_t dev_id_num; /* 设备 ID 号 */

    uint32_t soft_ver; /* 软件版本号 */
    uint32_t hard_ver; /* 硬件版本号 */

    uint8_t local_ip[4]; /* 本地 ip */
    uint8_t local_mask[4];
    uint8_t local_gateway[4];
    uint16_t local_port; /* 设备控制端口 */

    uint8_t upload_ip[4]; /* 上传目标 ip */
    uint16_t upload_port; /* 上传目标 端口 */
    uint16_t upload_point_clound_en : 3;
    uint16_t upload_dev_status_en : 1;
    uint16_t upload_alarm_info_en : 1;
    uint16_t upload_type_reserve : 11;

    float imu_offset_acc_x; /* imu 零位校准参数 */
    float imu_offset_acc_y;
    float imu_offset_acc_z;
    float imu_offset_gyro_x;
    float imu_offset_gyro_y;
    float imu_offset_gyro_z;

    float imu_waican[3][3];

    uint16_t dist_filter_en : 1;
    uint16_t energy_filter_en : 1;
    uint16_t dev_en_reserve : 14;
    uint8_t energy_filter_with_dist;
    uint8_t energy_filter_min;
    uint16_t dist_filter_min;
    uint16_t dist_filter_max;

    uint8_t alarm_event_io_map[16]; /* 报警事件与 IO 映射 */
    uint8_t alarm_event_io_output_mode_pnp_npn;

    uint16_t zone_detect_alg_para_min_frame;               /* 防区检测圈数滤波 */
    uint16_t zone_detect_alg_para_min_points;              /* 防区检测点数滤波 */
    uint16_t zone_detect_alg_abnormal_point_split_min_num; /* 异常点云分割点数 */
    float zone_detect_alg_point_clound_height;             /* 点云高度 */

    uint16_t frameSmooth_range;

    uint16_t crosstalk_en;
    uint16_t crosstalk_indoor_thr_1;
    uint16_t crosstalk_indoor_thr_2;
    uint16_t crosstalk_outdoor_thr_1;
    uint16_t crosstalk_outdoor_thr_2;

    uint8_t zone_detect_obj_io_map[4];

    uint8_t imu_type;
    uint8_t imu_acc_scale : 4;
    uint8_t imu_gyr_scale : 4;
    uint8_t imu_acc_filter : 4;
    uint8_t imu_gyr_filter : 4;
    uint8_t imu_odr;
    /* imu_type 为 板载 IMU 时有效 */
    uint8_t inner_imu_type; /* 0: LSM6D3TR, 1: ICM42688 , 2: ICM42652 */
    float imu_algo_filter_level;
    uint8_t reserve[404];
};
typedef struct
{
    char protocol_header[2]; // 协议头  默认LD
    uint16_t protocol_ver;   // 协议版本
    TIME_ST timestamp;
    unsigned char dev_sn[20];   // sn号
    unsigned char dev_type[16]; // 设备型号
    uint32_t software_ver;      // 软件版本号
    uint32_t dev_id;            // 设备ID
    uint8_t ip[4];
    uint8_t mask[4];
    uint8_t gateway[4];
    uint16_t listen_port; // 服务端口
    uint8_t remote_ip[4]; // 上传IP
    uint16_t remote_port; // 上传端口
    uint32_t status;      // 状态
    char reserve[20];
    uint32_t crc;
} DevHeart;

struct KeepAlive
{
    TIME_ST world_clock; // 纳秒级时间戳
    TIME_ST arrive_time; // 包数据主机到雷达的时间(内部使用)
    TIME_ST delay_time;  // 延迟时间(内部使用)
};

typedef struct
{
    union
    {
        struct
        {
            uint32_t zone_observe : 1;
            uint32_t zone_warning : 1;
            uint32_t zone_alarm : 1;
            uint32_t zone_cfg_invalid : 1;
            uint32_t tx_tem_h : 1;
            uint32_t rx_tem_h : 1;
            uint32_t tx_vol_h : 1;
            uint32_t tx_vol_l : 1;
            uint32_t rx_vol_h : 1;
            uint32_t rx_vol_l : 1;
            uint32_t no_cali_table : 1;
            uint32_t data_is_zero : 1;
            uint32_t no_imu_data : 1;

            uint32_t reserved : 19;
        };
        uint32_t events;
    };
} PROCOTOL_ALARM_EVENTS_ST;

typedef struct
{
    uint16_t id;              /* 计数 id */
    uint8_t actived_zone_num; /* 当前激活防区 0x00-0x0F */
    uint8_t reserve;
    PROCOTOL_ALARM_EVENTS_ST events; /* 报警事件 */
    uint8_t reserved[300];
} PROTOCOL_DATA_PACK_ALARM_ST;

#pragma pack(pop)
typedef void (*LidarCloudPointCallback)(uint32_t handle, const uint8_t dev_type, onePoi *data, uint16_t num, void *client_data);
typedef void (*LidarImuDataCallback)(uint32_t handle, const uint8_t dev_type, fs_lidar_imu_t *data, void *client_data);
typedef void (*LidarLogDataCallback)(uint32_t handle, const uint8_t dev_type, char *data, int len);

#endif
