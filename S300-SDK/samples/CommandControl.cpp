#include "../sdk/pacecatlidarsdk.h"

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
#if 1
	char lidar_addr[] = "192.168.158.98";
	int lidar_port = 6543;
	int listen_port = 6668;
	std::string adapter = "ens38";
#endif
#if 0
	char lidar_addr[] = "192.168.137.209";
	int lidar_port = 6543;
	int listen_port = 6002;
	std::string adapter = "ens38";
#endif

	PaceCatLidarSDK::getInstance()->Init(adapter);

	int devID = PaceCatLidarSDK::getInstance()->AddLidar(lidar_addr, lidar_port, listen_port);
	PaceCatLidarSDK::getInstance()->ConnectLidar(devID);
	PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);
	/*****************query lidar network(include check lidar isonline)**************************/
	NetWorkInfo info;
	bool ret = PaceCatLidarSDK::getInstance()->QueryBaseInfo(devID, info);
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
		printf("lidar_ip:%s lidar_imu_odrport:%d host_ip:%s  host_port:%d\n", lidar_ip_str.c_str(), info.lidar_port, host_ip_str.c_str(), info.host_port);
	}
	ret = PaceCatLidarSDK::getInstance()->SetWorking(devID,false);
	if(ret)
	{
		printf("set stop ok\n");
	}
	ret = PaceCatLidarSDK::getInstance()->SetWorking(devID,true);
	if(ret)
	{
		printf("set start ok\n");
	}

#if 0
	/*****************set lidar network(finish set,will restart, maximum wait time: 30 seconds)**************************/
	std::string set_lidar_ip="192.168.0.223";
	std::string set_lidar_mask="255.255.255.0";
	std::string set_lidar_gateway="192.168.0.1";
	uint16_t set_lidar_port=6543;//6001  6002 is used
	bool ret2 = PaceCatLidarSDK::getInstance()->SetLidarNetWork(devID, set_lidar_ip, set_lidar_mask,set_lidar_gateway,set_lidar_port);
	if (ret2)
	{
		printf("set lidar network ok, restart now\n");
	}
#endif
#if 0
	/*****************set lidar upload ip port(finish set,will restart, maximum wait time: 30 seconds)**************************/	

	std::string set_upload_ip="192.168.0.47";
	uint16_t set_upload_port=6668;
	bool ret2 = PaceCatLidarSDK::getInstance()->SetLidarUploadNetWork(devID, set_upload_ip, set_upload_port);
	if (ret2)
	{
		printf("set upload ip  port ok,restart now\n");
	}

#endif
#if 0
	/***********************set imu acc and gyro******************/
	bool ret_acc = PaceCatLidarSDK::getInstance()->SetImuAcc(devID, ACC_2);
	printf("set imu acc %s\n",ret_acc?"ok":"ng");
	bool ret_gyro = PaceCatLidarSDK::getInstance()->SetImuGyro(devID, GYRO_125);
	printf("set imu gyro %s\n",ret_gyro?"ok":"ng");

	bool ret_filterlevel = PaceCatLidarSDK::getInstance()->SetIMUFilterLevel(devID, SAMPLE_RATE_DIV4,SAMPLE_RATE_DIV4);
	printf("set imu filter level %s\n",ret_filterlevel?"ok":"ng");
	bool ret_samplerate = PaceCatLidarSDK::getInstance()->SetIMUSampleRate(devID, FREQ_200HZ);
	printf("set imu sample rate %s\n",ret_samplerate?"ok":"ng");

	bool ret_algofilterlevel = PaceCatLidarSDK::getInstance()->SetIMUAlgoFilterLevel(devID, 10);
	printf("set imu algo filter level %s\n",ret_algofilterlevel?"ok":"ng");

	ImuInfo imuinfo;
	bool ret_query_imu = PaceCatLidarSDK::getInstance()->QueryIMUInfo(devID,imuinfo);
	printf("query imu range:%s  acc_range:%u gyro_range:%u acc_filter_level:%u gyro_filter_level:%u sample_rate:%u  algo_filter_level:%f imu_model:%s\n",
		ret_query_imu?"ok":"ng",
		imuinfo.acc_range,
		imuinfo.gyro_range,
		imuinfo.acc_filter_level,
		imuinfo.gyro_filter_level,
		imuinfo.sample_rate,
		imuinfo.algo_filter_level,
		imuinfo.imu_model.c_str());
#endif

	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000 * 2));
#ifndef MODE_SIMPLE
		int state = PaceCatLidarSDK::getInstance()->QueryDeviceState(devID);
		if (state == ONLINE)
			printf("lidar online\n");
		else
			printf("lidar offline\n");
#endif
		printf("lidar timestamp sync:%d\n",PaceCatLidarSDK::getInstance()->SetTimeStampSync(devID));
	}
	PaceCatLidarSDK::getInstance()->deleteInstance();
}
