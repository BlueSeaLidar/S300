# 3 Interface description

This section focuses on the interfaces called externally in the PaceCatLidarSDK class.

***#include<pacecatlidarsdk.h>***

## Enumeration

```C++
enum LidarState
{
    OFFLINE = 0,
    ONLINE,
    QUIT
};

```

## Structure

```C++

struct CmdHeader{
    unsigned short sign; //Flags agreed upon with hardware
    unsigned short ver;//protocol version
    unsigned short msgid;//message id
    unsigned short cmd;//command
    unsigned int len;//command length
};

```

```C++
//Point cloud coordinate system
struct Point3D{
    float x;//unit:m
    float y;//unit:m
    float z;//unit:m
};

```

```C++

struct onePoi{
    uint16_t row_pos;//point index of row
    uint16_t col_pos;//point index of col
    float distance;//distance unit:cm
    uint32_t reflectivity; 
    Point3D pt3d;//coordinate system
    uint32_t peak0_pos;//
    uint32_t peak0_val;//
    uint32_t peak1_pos;//
    uint32_t peak1_val;//
    uint16_t envSumXi;
    uint32_t nbVal;
    uint32_t centDis;// 
    uint32_t riseDis;// 
    uint32_t firPeak;// 
    uint64_t timestamp; //unit:us
};

```

```C++

//public package head
typedef struct tag_soc_head{ // size 6
    uint16_t head;// fix，0x5A,0xA5
    uint8_t version_major; // versoion,latest 1
    uint8_t data_type;// data type: 0x01 0x0c 0x0d 0x0e
    uint16_t reserved;// 
} soc_head_t;

```

```C++
// point data package head (after public package head)
typedef struct tag_soc_protocol_data{// size 40
    uint8_t version_minor;// latest 0
    uint16_t data_sequence; // statistics on whether received data is lost or not
    uint16_t frame_cnt;// frame count
    uint8_t time_type;// timestamp type
    uint16_t col;// current package index col
    uint16_t row;// current package index row
    uint16_t dot_num;// current package include point num
    uint8_t type;// point cloud data type（Distance/XYZ/(Distance+XYZ) ）+ reflectivity + default XYZ
    uint8_t echo_num;//default dual echo, maximum supported number of echoes is 4 echoes
    uint8_t frame_flag;// 0 frame head 1 frame body 2:frame tail   
    uint8_t reserved[11];
    uint8_t crc8;
    uint32_t crc32;
    uint64_t timestamp;// unit:us
    uint8_t data[0];// point data pointer
} soc_protocol_data_t;

```

```C++

// point data(after point data package head)
typedef struct tag_soc_dis_pt_data{
    uint16_t distance; //uint:cm 
    int16_t x;//uint:mm    factor:0.5
    int16_t y;//uint:mm    factor:0.5
    int16_t z;//uint:mm    factor:0.5
    uint8_t reflect; 
    uint8_t label;//Signs for some special scenarios
} soc_dis_pt_data_t;

```

```C++
//imu data head(after public package head)
typedef struct {
    uint8_t  crc8_header;//not use
    uint8_t  crc8_imu_dat;//not use
    uint8_t  version;//imu version   2/3
    uint16_t len;//package length
    uint8_t  time_type;//not use
    int64_t  timestamp;//
    int16_t  temperature;//unit: 1/256 ℃
    uint8_t  data[0];
} fs_lidar_imu_header_t;

```

```C++
//imu data(after imu data head)  fs_lidar_imu_header_t->version = 2
typedef struct {
    float gyro_x; // gyroscope angular velocity x:°/s
    float gyro_y; // gyroscope angular velocity y:°/s
    float gyro_z; // gyroscope angular velocity z:°/s
    float acc_x;  // acceleration:g
    float acc_y;  // acceleration:g
    float acc_z;  // acceleration:g
} fs_lidar_imu_v2;
```

```C++
//imu data(after imu data head) fs_lidar_imu_header_t->version = 3
typedef struct {
    float pitch;//rotate x  degree
    float roll;//rotate y   degree
    float yaw;//rotate z    degree
    float gyro_x; // gyroscope angular velocity x:°/s
    float gyro_y; // gyroscope angular velocity y:°/s
    float gyro_z; // gyroscope angular velocity z:°/s
    float acc_x;  // acceleration:g
    float acc_y;  // acceleration:g
    float acc_z;  // acceleration:g
} fs_lidar_imu_v3;
    
```

```C++
//imu integrate publish 
typedef struct {
    int64_t  timestamp;
    uint8_t  version;//imu version(different versions represent different IMU hardware.)
    float pitch;//rotate x   degree
    float roll;//rotate y    degree
    float yaw;//rotate z     degree
    float gyro_x; // gyroscope angular velocity x:°/s
    float gyro_y; // gyroscope angular velocity y:°/s
    float gyro_z; // gyroscope angular velocity z:°/s
    float acc_x;  // acceleration:g
    float acc_y;  // acceleration:g
    float acc_z;  // acceleration:g
    
} fs_lidar_imu_t;

```

```C++
//device parameter
struct DEV_CFG_ST
    {
        uint16_t cfg_ver;//version

        char dev_type[16];//model :LDS-S00-E
        char dev_sn[32];//device SN
        uint32_t dev_id_num; //device ID

        uint32_t soft_ver;
        uint32_t hard_ver;

        uint8_t local_ip[4]; //device ip
        uint8_t local_mask[4]; //device mask
        uint8_t local_gateway[4];//device gateway
        uint16_t local_port;//device  listen port

        uint8_t upload_ip[4];
        uint16_t upload_port;
        uint16_t upload_point_clound_en : 3;//data echo mode 0 no upload 1 single echo 360*150;2 single echo filter 360*75
        uint16_t upload_dev_status_en : 1;
        uint16_t upload_alarm_info_en : 1;
        uint16_t upload_type_reserve : 11;
        uint8_t reserve[524];

    };

```

```C++
//heartbeat package  
typedef struct{
    char protocol_header[2];//default LD
    uint16_t protocol_ver;//=
    TIME_ST timestamp;//seconds + nanoseconds (data after seconds converted to nanoseconds)
    unsigned char dev_sn[20];//sn
    unsigned char dev_type[16];//model
    uint32_t software_ver;
    uint32_t dev_id;
    uint8_t  ip[4];
    uint8_t  mask[4];
    uint8_t  gateway[4];
    uint16_t listen_port;
    uint8_t  remote_ip[4];
    uint16_t remote_port;
    uint32_t status;
    char reserve[20];
    uint32_t crc;
} DevHeart;

```

```C++
//define alarm events 
typedef struct
{
  union
  {
    struct
    {
        uint32_t zone_observe : 1;    
        uint32_t zone_warning : 1;     
        uint32_t zone_alarm : 1;
        uint32_t zone_cfg_invalid : 1;//no  zone set
        uint32_t tx_tem_h : 1;// transmitter high temperatures            
        uint32_t rx_tem_h : 1;//receiver  high temperatures          
        uint32_t tx_vol_h : 1;//transmitter high voltage       
        uint32_t tx_vol_l : 1;//transmitter low voltage        
        uint32_t rx_vol_h : 1;//receiver  high voltage           
        uint32_t rx_vol_l : 1;//receiver  low voltage               
        uint32_t no_cali_table : 1;
        uint32_t data_is_zero : 1;//point data distance  is zero  in one frame     
        uint32_t no_imu_data : 1; 

      uint32_t reserved : 19;           
    };
    uint32_t events;
  };
}__attribute__ ((packed)) PROCOTOL_ALARM_EVENTS_ST;


```

```C++
//alarm package
typedef struct
{
  uint16_t id; //id   incremental counting
  uint8_t actived_zone_num;// range:0x00-0x0F
  uint8_t reserve;
  PROCOTOL_ALARM_EVENTS_ST events;//alarm events 
  uint8_t reserved[300];
}__attribute__ ((packed)) PROTOCOL_DATA_PACK_ALARM_ST;

```

```C++
//Network Information  
struct NetWorkInfo {
    uint32_t lidar_ip;
    uint32_t host_ip;
    uint16_t lidar_port;
    uint16_t host_port;
};

```

## Function

```C++
@Name    Init
@Brief   Class initialization, mainly to enable heartbeat packet threads
@Param   std::string adapter         Current network adapter name (Bind network adapter to prevent abnormal reception of heartbeat packets on multiple network adapters)
@Return  void
@Note    The first step in running the example

```

```C++
@Name    Uninit
@Brief   Class counter-initialization, mainly to close all worker threads
@Return  void
@Note    NULL

```

```C++
@Name    SetPointCloudCallback
@Brief   Setting up callback functions for point cloud data
@Param   int ID                      lidar ID
@Param   LidarCloudPointCallback cb  Pointer to the callback function
@Param   void* client_data           Data pointers that the user needs to use globally
@Return  bool
@Note    The third step in running the example
```

```C++
@Name    SetImuDataCallback
@Brief   Setting up callback functions for imu
@Param   int ID                      lidar ID
@Param   LidarImuDataCallback cb     Pointer to the callback function
@Param   void* client_data           Data pointers that the user needs to use globally
@Return  bool
@Note    The third step in running the example
```

```C++
@Name    SetLogDataCallback
@Brief   Setting up callback functions for log
@Param   int ID                      lidar ID
@Param   LidarImuDataCallback cb     Pointer to the callback function
@Param   void* client_data           Data pointers that the user needs to use globally
@Return  bool
@Note    The third step in running the example
```

```C++
@Name    WritePointCloud
@Brief   write pointcloud data
@Param   int ID                      
@Param   const uint8_t dev_type      
@Param   onePoi *data                point cloud pointer
@Param   int     num                 point cloud num
@Return  void
@Note    inside use
```

```C++
@Name    WriteImuData
@Brief   write imu data
@Param   int ID                      
@Param   const uint8_t dev_type      
@Param   fs_lidar_imu_t *data      imu pointer 
@Return  void
@Note    inside use
```

```C++
@Name    WriteLogData
@Brief   write log data
@Param   int ID                      
@Param   const uint8_t dev_type      
@Param   char* *data                 
@Param   int len                     
@Return  void
@Note    inside use
```

```C++
@Name    AddLidar
@Brief   add lidar
@Param   std::string lidar_ip        lidar ip
@Param   int lidar_port              lidar port
@Param   int listen_port             local listen port
@Return  int                         lidar ID
@Note    The secord step
```

```C++
@Name    ConnectLidar
@Brief   Connects to lidar, sends commands, receives data.
@Param   int ID                      lidar ID
@Return  bool
@Note   
```

```C++
@Name    DisconnectLidar
@Brief   disconnect lidar
@Param   int ID                      lidar ID
@Return  bool
@Note   
```

```C++
@Name    QueryBaseInfo
@Brief   query lidar base info
@Param   int ID                      lidar ID
@Param   NetWorkInfo info               
@Return  bool
@Note   
```

```C++
@Name    QueryVersion
@Brief   query lidar version
@Param   int ID                      lidar ID
@Param   VersionInfo info            
@Return  bool
@Note   
```

```C++
@Name    QueryDeviceState
@Brief   query device state
@Param   int ID                      lidar ID
@Return  int LidarState
@Note   
```

```C++
@Name    SetLidarNetWork
@Brief   set lidar network(Not open)
@Param   int ID                      lidar ID
@Param   std::string ip              ip
@Param   std::string mask            mask
@Param   std::string gateway         gateway
@Param   uint16_t port               port
@Return  bool                        
@Note   
```

```C++
@Name    SetLidarUploadNetWork
@Brief   set lidar upload network(Not open)
@Param   int ID                      lidar ID
@Param   std::string upload_ip       upload ip
@Param   uint16_t upload_port        upload port
@Return  bool                        
@Note   
```

```C++
@Name    SetYawAngle
@Brief   set yaw
@Param   int ID                     lidar ID
@Param   int yaw                    Yaw angle (version 2 unit:rad/s version 3 unit:°/s)        
@Return  bool                        
@Note   
```

```C++
@Name    SetWorking
@Brief   Setting lidar start stop
@Param   int ID                      lidar ID
@Param   bool isrun                  start stop
@Return  bool                        
@Note   
```

```C++
@Name  QueryIDByIp
@Brief   query id by lidar ip
@Param   const char* lidar_ip         lidar ip
@Return   int                         lidar ID                  
@Note   
```

```C++
@Name    GetConfig
@Brief   Get configuration information based on lidar ID
@Param   int ID                       lidar ID
@Return  RunConfig                   lidar config                       
@Note   
```
