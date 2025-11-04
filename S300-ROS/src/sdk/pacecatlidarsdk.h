// M300_SDK.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。
#pragma once

#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include<string>
#ifdef _WIN32
#pragma warning(disable : 4996)
#pragma warning(disable : 4244)
#include <winsock2.h>
#include <iphlpapi.h>
#pragma comment(lib, "iphlpapi.lib")
#pragma comment(lib, "ws2_32.lib")
typedef uint32_t in_addr_t;
#else
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#endif
#include"protocol.h"
#include"../3rdparty/concurrentqueue/concurrentqueue.h"
using namespace moodycamel;
#define S300_E_SDKVERSION "V1.7.1_2025102201" // SDK版本号


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
	std::string motor_ver;
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
	std::string adapter;
}HeartInfo;

struct NetWorkInfo {
    uint32_t lidar_ip;
    uint32_t host_ip;
    uint16_t lidar_port;
    uint16_t host_port;
};
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
	std::thread  thread_recv;
	std::thread  thread_parse;
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
	int udp_cmd_fd;
	int udp_data_fd;
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
	DEV_CFG_ST dev_cfg;
	ConcurrentQueue<std::string> data_queue;

};
class PaceCatLidarSDK
{
public:
	static PaceCatLidarSDK *getInstance();
	static void deleteInstance();

	void Init(std::string adapter);
	void Uninit();

	/*
	 *	callback function  get pointcloud data  imu data  logdata
	 */
	bool SetPointCloudCallback(int ID, LidarCloudPointCallback cb, void* client_data);
	bool SetImuDataCallback(int ID, LidarImuDataCallback cb, void* client_data);
	bool SetLogDataCallback(int ID, LidarLogDataCallback cb, void* client_data);

	void WritePointCloud(int ID, const uint8_t dev_type, onePoi *data,uint16_t num);
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
	bool QueryBaseInfo(int ID, NetWorkInfo &info);

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
	bool SetLidarNetWork(int ID,std::string ip, std::string mask, std::string gateway, uint16_t port);

	/*
	 *	set lidar    upload ip    upload port     fixed upload or not
	 */
	bool SetLidarUploadNetWork(int ID, std::string upload_ip, uint16_t upload_port);

	/*
	*	set timestamp sync
	*/
	bool SetTimeStampSync(int ID);
	/*
	*	set lidar start/stop
	*/
	bool SetWorking(int ID,bool isrun);

	int QueryIDByIp(std::string ip);
	RunConfig*getConfig(int ID);
protected:

private:
	void UDPThreadRecv(int id);
	void UDPThreadParse(int id);

	void HeartThreadProc(HeartInfo &heartinfo);

	std::string GetErrorEvent(PROCOTOL_ALARM_EVENTS_ST *syseventlog);
	in_addr_t get_interface_ip(const char *ifname);
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

	HeartInfo m_heartinfo;
	std::thread m_heartthread;
	
};









