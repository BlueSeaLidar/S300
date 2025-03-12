#include"../sdk/sdk.h"

onePoi *p_point_data=NULL;
void PointCloudCallback(uint32_t handle, const uint8_t dev_type, onePoi* data, void* client_data) {
	if (data == nullptr) {
		return;
	}
	if(p_point_data==NULL)
    		p_point_data = new onePoi[HEIGHT * WIDTH];
    	
    	memcpy(p_point_data,data,sizeof(onePoi)*HEIGHT * WIDTH);
    	
	
	// for(int i=0;i<WIDTH*HEIGHT;i++)
	// {
	// 	printf("point cloud handle: %u, idx:%d X: %lf, Y: %lf, Z: %lf, timestamp: %ld\n",
	// 	handle, i,data[i].pt3d.x, data[i].pt3d.y, data[i].pt3d.z, data[i].timestamp);
	// }
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type, fs_lidar_imu_t* data, void* client_data) {
	if (data == nullptr) {
		return;
	}
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
	char lidar_addr[] = "192.168.137.200";
	int lidar_port = 6001;
	int listen_port = 6002;
	PaceCatLidarSDK::getInstance()->Init();
	int devID = PaceCatLidarSDK::getInstance()->AddLidar(lidar_addr, lidar_port, listen_port);

	PaceCatLidarSDK::getInstance()->SetPointCloudCallback(devID,PointCloudCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetImuDataCallback(devID, ImuDataCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);
	PaceCatLidarSDK::getInstance()->ConnectLidar(devID);


	//multiple lidars  ,please make sure lidar ip and   localport  is must be not same 

	// char lidar_addr2[] = "192.168.1.10";
	// int lidar_port2 = 6543;
	// int listen_port2 = 6669;
	// int devID2 = PaceCatLidarSDK::getInstance()->AddLidar(lidar_addr2, lidar_port2, listen_port2); 
	// PaceCatLidarSDK::getInstance()->SetPointCloudCallback(devID2, PointCloudCallback, nullptr);
	// PaceCatLidarSDK::getInstance()->SetImuDataCallback(devID2, ImuDataCallback, nullptr);
	// PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID2, LogDataCallback, nullptr);
	// PaceCatLidarSDK::getInstance()->ConnectLidar(devID2);


	printf("use cloudpoint data  reference function  PointCloudCallback \n");
	printf("use imu data  reference function  ImuDataCallback \n");
	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	PaceCatLidarSDK::getInstance()->Uninit();
}
