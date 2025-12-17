#include"../sdk/pacecatlidarsdk.h"
#include"../sdk/global.h"
onePoi *p_point_data=NULL;
uint64_t lasttime=0;
int idx=0;
void PointCloudCallback(uint32_t handle, const uint8_t dev_type, onePoi* data, uint16_t num,void* client_data) {
	if (data == nullptr) {
		return;
	}
	if(p_point_data==NULL)
    	p_point_data = new onePoi[num];
    	
    memcpy(p_point_data,data,sizeof(onePoi)*num);

	//printf("%d\n",num);
	// if(idx==0)
	// {
	// 	lasttime= SystemAPI::GetTimeStamp_us(true);
	// }

	// if(idx%20==0 && idx!=0)
	// {
	// 	uint64_t timetsamp = SystemAPI::GetTimeStamp_us(true);
	// 	printf("111:%d %d\n",timetsamp-lasttime,idx);
	// 	lasttime=timetsamp;
	// }
	// idx++;
	
	// for(int i=0;i<num;i++)
	// {
	// 	printf("point cloud handle: %u, idx:%d X: %lf, Y: %lf, Z: %lf, timestamp: %ld\n",
	// 	handle, i,data[i].pt3d.x, data[i].pt3d.y, data[i].pt3d.z, data[i].timestamp);
	// }
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type, fs_lidar_imu_t* data, void* client_data) {
	if (data == nullptr) {
		return;
	}
	// if(idx==0)
	// {
	// 	lasttime= SystemAPI::GetTimeStamp_us(true);
	// }

	// if(idx%10==0 && idx!=0)
	// {
	// 	uint64_t timetsamp = SystemAPI::GetTimeStamp_us(true);
	// 	printf("111:%d %d\n",timetsamp-lasttime,idx);
	// 	lasttime=timetsamp;
	// }
	// idx++;
	// printf("Imu data callback handle:%u, acc_x:%f, acc_y:%f acc_z:%f gyro_x:%f gyro_y:%f gyro_z:%f\n",
	// 	handle, data->acc_x,data->acc_y,data->acc_z,data->gyro_x,data->gyro_y,data->gyro_z);
}

void LogDataCallback(uint32_t handle, const uint8_t dev_type, char* data, int len) {
	if (data == nullptr) {
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

	PaceCatLidarSDK::getInstance()->SetPointCloudCallback(devID,PointCloudCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetImuDataCallback(devID, ImuDataCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);
	PaceCatLidarSDK::getInstance()->ConnectLidar(devID);

#if 0
	//multiple lidars  ,please make sure lidar ip and   localport  is must be not same 
	char lidar_addr2[] = "192.168.1.10";
	int lidar_port2 = 6543;
	int listen_port2 = 6669;
	int devID2 = PaceCatLidarSDK::getInstance()->AddLidar(lidar_addr2, lidar_port2, listen_port2); 
	PaceCatLidarSDK::getInstance()->SetPointCloudCallback(devID2, PointCloudCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetImuDataCallback(devID2, ImuDataCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID2, LogDataCallback, nullptr);
	PaceCatLidarSDK::getInstance()->ConnectLidar(devID2);
#endif

	printf("use cloudpoint data  reference function  PointCloudCallback \n");
	printf("use imu data  reference function  ImuDataCallback \n");
	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	PaceCatLidarSDK::getInstance()->deleteInstance();
}
