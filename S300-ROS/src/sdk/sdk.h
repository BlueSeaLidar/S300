// M300_SDK.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。
#pragma once

#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include<string>
#include"protocol.h"
#include "FsTcpTransControl.h"
#define S300_E_SDKVERSION "V1.0_20250107" // SDK版本号


typedef struct
{
	std::string uuid;
	std::string model;
	std::string lidarip;
	std::string lidarmask;
	std::string lidargateway;
	uint16_t lidarport;
	std::string uploadip;
	uint16_t uploadport;
	uint8_t uploadfix;

}BaseInfo;

typedef struct
{
	//std::string mcu_ver;
	//std::string motor_ver;
	std::string software_ver;
}VersionInfo;

typedef struct
{
	std::string sn;
	std::string ip;
	int port;
	uint64_t timestamp;
	bool state;
	int flag;
}ConnectInfo;
typedef struct
{
	std::vector<ConnectInfo>lidars;
	int code;
	std::string value;
	bool isrun;
}HeartInfo;

enum LidarState
{
	OFFLINE = 0,
	ONLINE,
	QUIT
};
enum LidarAction
{
	NONE,
	FINISH,
	START,
	STOP,
	GET_PARAMS,
	GET_VERSION,
	SET_NETWORK,
	SET_UPLOAD_NETWORK,
	SET_UPLOAD_FIX,
	UPGRADE
};
enum LidarMsg
{
	MSG_DEBUG,
	MSG_WARM,
	MSG_ERROR

};

//运行配置 
struct RunConfig
{
	int ID;
	std::thread  thread_subData;
	//std::thread  thread_pubCloud;
	//std::thread  thread_pubImu;
	LidarCloudPointCallback  cb_cloudpoint;
	void *cloudpoint;
	LidarImuDataCallback cb_imudata;
	void *imudata;
	LidarLogDataCallback cb_logdata;
	void*logdata;
	int run_state;
	std::string lidar_ip;
	int lidar_port;
	int listen_port;
	//std::vector<LidarCloudPointData> cloud_data;
	//std::queue<IIM42652_FIFO_PACKET_16_ST> imu_data;
	uint32_t frame_cnt;
	uint64_t frame_firstpoint_timestamp;  //everyframe  first point timestamp
	//uint64_t frame_firstpoint_timestamp_cache;
	int action;
	int send_len;
	std::string send_buf;
	int recv_len;
	std::string recv_buf;

};
class PaceCatLidarSDK
{
public:
	static PaceCatLidarSDK *getInstance();
	static void deleteInstance();

	void Init();
	void Uninit();

	/*
	 *	callback function  get pointcloud data  imu data  logdata
	 */
	bool SetPointCloudCallback(int ID, LidarCloudPointCallback cb, void* client_data);
	bool SetImuDataCallback(int ID, LidarImuDataCallback cb, void* client_data);
	bool SetLogDataCallback(int ID, LidarLogDataCallback cb, void* client_data);

	void WritePointCloud(int ID, const uint8_t dev_type, onePoi *data);
	void WriteImuData( int ID, const uint8_t dev_type, fs_lidar_imu_t* data);
	void WriteLogData(int ID, const uint8_t dev_type, char* data, int len);


	/*
	 *	add lidar by lidar ip    lidar port    local listen port
	 */
	int AddLidar(std::string lidar_ip, int lidar_port, int listen_port);
	/*
	 *	connect lidar     send cmd/parse recvice data
	 */
	bool ConnectLidar(int ID);
	/*
	 *	disconnect lidar,cancel listen lidar
	 */
	bool DisconnectLidar(int ID);

	/*
	 *	query connect lidar base info
	 */
	bool QueryBaseInfo(int ID, std::string lidarip,fs_ipport_t &info);

	/*
	 *	query connect lidar version
	 */
	bool QueryVersion(int ID, VersionInfo &info);

	/*
	 *	use by lidar heart  query lidar  state
	 */
	int QueryDeviceState(int ID);

	/*
	 *	set lidar    ip  mask  gateway  receive port
	 */
	bool SetLidarNetWork(int ID, std::string in_lidar_ip,uint16_t in_lidar_port,std::string in_host_ip,uint16_t in_host_port);



	int QueryIDByIp(std::string ip);
	RunConfig*getConfig(int ID);
protected:

private:
	
private:
	static PaceCatLidarSDK *m_sdk;
	PaceCatLidarSDK();
	~PaceCatLidarSDK();

	int m_idx;
	std::vector<RunConfig*> m_lidars;
	uint64_t m_npoint = 0;
	uint64_t m_npub = 0;
	bool bottom_ccw = false;
	uint64_t last_ns = 0;
	int64_t sidx = 0;
	int64_t imu_idx = 0;
	int m_currentframeidx{ 0 };
};

void UDPThreadProc(int id);









