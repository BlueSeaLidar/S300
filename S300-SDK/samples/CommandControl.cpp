#include "../sdk/sdk.h"

void LogDataCallback(uint32_t handle, const uint8_t dev_type, char *data, int len)
{
	if (data == nullptr)
	{
		return;
	}
	printf("ID::%d print level:%d msg:%s\n", handle, dev_type, data);
}

int main()
{
	char lidar_addr[] = "192.168.137.200";
	int lidar_port = 6001;
	int listen_port = 6002;

	PaceCatLidarSDK::getInstance()->Init();
	int devID = PaceCatLidarSDK::getInstance()->AddLidar(lidar_addr, lidar_port, listen_port);
	PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);

	/***********************check lidar isonline**********************************/

	int state = PaceCatLidarSDK::getInstance()->QueryDeviceState(devID);
	if (state == ONLINE)
	{
		printf("lidar online\n");
	}
	else
	{
		printf("lidar offline\n");
		return -1;
	}

	/*****************query lidar network(include check lidar isonline)**************************/
	fs_ipport_t info;
	bool ret = PaceCatLidarSDK::getInstance()->QueryBaseInfo(devID, lidar_addr, info);
	if (ret)
	{
		struct in_addr ip_addr;
		ip_addr.s_addr = info.lidar_ip;
		struct in_addr ip_addr2;
		ip_addr2.s_addr = info.host_ip;
		std::string lidar_ip_str;
		std::string host_ip_str;
		const char *ip_str_result = inet_ntoa(ip_addr);
		lidar_ip_str.assign(ip_str_result, 16);
		const char *ip_str_result2 = inet_ntoa(ip_addr2);
		host_ip_str.assign(ip_str_result2, 16);
		printf("lidar_ip:%s lidar_port:%d host_ip:%s  host_port:%d\n", lidar_ip_str.c_str(), info.lidar_port, host_ip_str.c_str(), info.host_port);
	}

	/*****************set lidar network(include check lidar isonline)**************************/
	std::string in_lidar_ip="192.168.0.137";
	std::string in_host_ip="192.168.0.123";
	uint16_t in_lidar_port=6543;
	uint16_t in_host_port=6669;
	bool ret2 = PaceCatLidarSDK::getInstance()->SetLidarNetWork(devID, in_lidar_ip, in_lidar_port,in_host_ip,in_host_port);
	if (ret2)
	{
		printf("set lidar network ok,please restart power\n");
	}
	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	PaceCatLidarSDK::getInstance()->Uninit();
}