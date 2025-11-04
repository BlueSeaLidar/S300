// M300_SDK.cpp: 定义应用程序的入口点。
//
#include "pacecatlidarsdk.h"
#include "global.h"
#include <fstream>
#ifdef _WIN32
#pragma warning(disable : 4996)
#pragma warning(disable : 4244)
#endif
#define M_PI 3.14159265358979323846

PaceCatLidarSDK *PaceCatLidarSDK::m_sdk = new (std::nothrow) PaceCatLidarSDK();
PaceCatLidarSDK *PaceCatLidarSDK::getInstance()
{
	return m_sdk;
}

void PaceCatLidarSDK::deleteInstance()
{
	if (m_sdk)
	{
		delete m_sdk;
		m_sdk = NULL;
	}
}
PaceCatLidarSDK::PaceCatLidarSDK()
{
}

PaceCatLidarSDK::~PaceCatLidarSDK()
{
	m_heartinfo.isrun = false;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		m_lidars[i]->run_state = OFFLINE;
		SystemAPI::closefd(m_lidars[i]->udp_data_fd, true);
		SystemAPI::closefd(m_lidars[i]->udp_cmd_fd, true);
	}
}
bool PaceCatLidarSDK::SetPointCloudCallback(int ID, LidarCloudPointCallback cb, void *client_data)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->cb_cloudpoint = cb;
	lidar->cloudpoint = client_data;
	return true;
}
bool PaceCatLidarSDK::SetImuDataCallback(int ID, LidarImuDataCallback cb, void *client_data)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->cb_imudata = cb;
	lidar->imudata = (char *)client_data;
	return true;
}
bool PaceCatLidarSDK::SetLogDataCallback(int ID, LidarLogDataCallback cb, void *client_data)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->cb_logdata = cb;
	lidar->logdata = (char *)client_data;
	return true;
}

void PaceCatLidarSDK::WritePointCloud(int ID, const uint8_t dev_type, onePoi *data, uint16_t num)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return;

	if (lidar->cb_cloudpoint != nullptr)
		lidar->cb_cloudpoint(ID, dev_type, data, num, lidar->cloudpoint);
}

void PaceCatLidarSDK::WriteImuData(int ID, const uint8_t dev_type, fs_lidar_imu_t *data)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return;
	if (lidar->cb_imudata != nullptr)
		lidar->cb_imudata(ID, dev_type, data, lidar->imudata);
}

void PaceCatLidarSDK::WriteLogData(int ID, const uint8_t dev_type, char *data, int len)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return;
	if (lidar->cb_logdata != nullptr)
		lidar->cb_logdata(ID, dev_type, data, len);
}

void PaceCatLidarSDK::Init(std::string adapter)
{
	m_heartinfo.code = 0;
	m_heartinfo.isrun = true;
	m_heartinfo.adapter = adapter;
	m_heartthread = std::thread(&PaceCatLidarSDK::HeartThreadProc, PaceCatLidarSDK::getInstance(), std::ref(m_heartinfo));
	m_heartthread.detach();
}

void PaceCatLidarSDK::Uninit()
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		m_lidars.at(i)->run_state = QUIT;
		SystemAPI::closefd(m_lidars[i]->udp_cmd_fd, true);
		SystemAPI::closefd(m_lidars[i]->udp_data_fd, true);
	}
}
int PaceCatLidarSDK::AddLidar(std::string lidar_ip, int lidar_port, int listen_port)
{
	RunConfig *cfg = new RunConfig;
	cfg->lidar_ip = lidar_ip.c_str();
	cfg->lidar_port = lidar_port;
	cfg->listen_port = listen_port;
	cfg->ID = m_lidars.size();
	cfg->run_state = ONLINE;
	cfg->frame_cnt = 0;
	cfg->cb_cloudpoint = NULL;
	cfg->cb_imudata = NULL;
	cfg->cb_logdata = NULL;
	cfg->frame_firstpoint_timestamp = 0;
	cfg->data_queue = ConcurrentQueue<std::string>(100);
	m_lidars.push_back(cfg);
	return cfg->ID;
}

bool PaceCatLidarSDK::ConnectLidar(int ID)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return false;
	lidar->udp_data_fd = SystemAPI::open_socket_port(lidar->listen_port, false);
	if (lidar->udp_data_fd <= 0)
	{
		std::string err = "listen port open failed";
		WriteLogData(lidar->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		return false;
	}
	lidar->udp_cmd_fd = SystemAPI::open_socket_port(lidar->lidar_port, false);
	if (lidar->udp_cmd_fd <= 0)
	{
		std::string err = "udp cmd port open failed";
		WriteLogData(lidar->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		SystemAPI::closefd(lidar->udp_data_fd, true);
		return false;
	}
	// 先获取设备信息，保存
	int index = 5;
	bool isok = false;
	char str = 0x00;
	std::string send_buf = BaseAPI::generate_cmd(0x00, rand(), 0x01, 1, &str);
	int recv_len;
	char recv_buf[1024] = {0};
	for (int i = 0; i < index; i++)
	{
		bool ret = CommunicationAPI::udp_talk_pack(lidar->udp_cmd_fd, lidar->lidar_ip.c_str(), lidar->lidar_port, send_buf.size(), send_buf.c_str(), recv_len, recv_buf);
		if (ret)
		{
			if (recv_len == sizeof(DEV_CFG_ST))
			{
				DEV_CFG_ST *cfg = (DEV_CFG_ST *)recv_buf;
				memcpy(&lidar->dev_cfg, cfg, recv_len);
				isok = true;
				break;
			}
		}
	}
	if (!isok)
	{
		std::string err = "query lidar err";
		WriteLogData(lidar->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		SystemAPI::closefd(lidar->udp_data_fd, true);
		SystemAPI::closefd(lidar->udp_cmd_fd, true);
		return false;
	}
	// 打印版本信息以及SN号
	VersionInfo info;
	QueryVersion(ID, info);
	std::string err = "lidar sn:" + BaseAPI::stringfilter(lidar->dev_cfg.dev_sn, 32) + "softwar_ver:" + info.software_ver + "motor_ver:" + info.motor_ver;
	WriteLogData(lidar->ID, MSG_INFO, (char *)err.c_str(), err.size());
	lidar->thread_recv = std::thread(&PaceCatLidarSDK::UDPThreadRecv, PaceCatLidarSDK::getInstance(), ID);
	lidar->thread_recv.detach();

	lidar->thread_parse = std::thread(&PaceCatLidarSDK::UDPThreadParse, PaceCatLidarSDK::getInstance(), ID);
	lidar->thread_parse.detach();

	return true;
}

bool PaceCatLidarSDK::DisconnectLidar(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			m_lidars.at(i)->run_state = OFFLINE;
			return true;
		}
	}
	return false;
}

bool PaceCatLidarSDK::QueryBaseInfo(int ID, NetWorkInfo &info)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return false;

	char str = 0x00;
	std::string send_buf = BaseAPI::generate_cmd(0x00, rand(), 0x01, 1, &str);

	int recv_len = 0;
	char recv_buf[1024] = {0};
	bool ret = CommunicationAPI::udp_talk_pack(lidar->udp_cmd_fd, lidar->lidar_ip.c_str(), lidar->lidar_port, send_buf.size(), send_buf.c_str(), recv_len, recv_buf);
	if (ret)
	{
		if (recv_len == sizeof(DEV_CFG_ST))
		{
			DEV_CFG_ST *cfg = (DEV_CFG_ST *)recv_buf;
			memcpy(&info.host_ip, cfg->upload_ip, 4);
			info.host_port = cfg->upload_port;
			memcpy(&info.lidar_ip, cfg->local_ip, 4);
			info.lidar_port = cfg->local_port;
			return true;
		}
	}
	return false;
}
bool PaceCatLidarSDK::QueryVersion(int ID, VersionInfo &info)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return false;

	int hard_maj = lidar->dev_cfg.hard_ver >> 26;
	int hard_min = lidar->dev_cfg.hard_ver >> 20 & 0x003F;
	int hard_date = lidar->dev_cfg.hard_ver & 0x000FFFFF;

	int soft_maj = lidar->dev_cfg.soft_ver >> 26;
	int soft_min = lidar->dev_cfg.soft_ver >> 20 & 0x003F;
	int soft_date = lidar->dev_cfg.soft_ver & 0x000FFFFF;
	info.motor_ver = std::to_string(hard_maj) + "_" + std::to_string(hard_min) + "_" + std::to_string(hard_date);
	info.software_ver = std::to_string(soft_maj) + "_" + std::to_string(soft_min) + "_" + std::to_string(soft_date);
	return true;
}
int PaceCatLidarSDK::QueryDeviceState(int ID)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return OFFLINE;

	std::string ip = lidar->lidar_ip;

	for (unsigned int i = 0; i < m_heartinfo.lidars.size(); i++)
	{
		std::string tmpip = m_heartinfo.lidars[i].ip;
		if (ip == tmpip)
		{
			return m_heartinfo.lidars[i].state;
		}
	}
	return OFFLINE;
}

bool PaceCatLidarSDK::SetLidarNetWork(int ID, std::string ip, std::string mask, std::string gateway, uint16_t port)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return OFFLINE;

	char result[1024] = {0};
	bool ret = BaseAPI::checkAndMerge(1, (char *)ip.c_str(),
									  (char *)mask.c_str(),
									  (char *)gateway.c_str(),
									  port,
									  result);

	if (!ret)
		return false;

	char cmd[14] = {0};
	std::vector<uint8_t> iplist = BaseAPI::split_ip_to_bytes_method3(ip);
	std::vector<uint8_t> masklist = BaseAPI::split_ip_to_bytes_method3(mask);
	std::vector<uint8_t> gatewaylist = BaseAPI::split_ip_to_bytes_method3(gateway);

	for (uint8_t i = 0; i < 4; i++)
		cmd[i] = iplist[i];
	for (uint8_t i = 0; i < 4; i++)
		cmd[i + 4] = masklist[i];
	for (uint8_t i = 0; i < 4; i++)
		cmd[i + 8] = gatewaylist[i];

	memcpy(cmd + 12, &port, 2);
	std::string send_buf = BaseAPI::generate_cmd2(2, 14, cmd);
	int recv_len;
	char recv_buf[1024] = {0};
	ret = CommunicationAPI::udp_talk_pack(lidar->udp_cmd_fd, lidar->lidar_ip.c_str(), lidar->lidar_port, send_buf.size(), send_buf.c_str(), recv_len, recv_buf);
	if (ret)
	{
		printf("SetLidarNetWork:%s\n",recv_buf);
		return ret;
	}

	return false;
}

bool PaceCatLidarSDK::SetLidarUploadNetWork(int ID, std::string upload_ip, uint16_t upload_port)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return OFFLINE;

	char result[1024] = {0};
	bool ret = BaseAPI::checkAndMerge(0, (char *)upload_ip.c_str(),
									  (char *)" ", (char *)" ",
									  upload_port,
									  result);

	if (!ret)
		return false;

	char cmd[6] = {0};
	std::vector<uint8_t> iplist = BaseAPI::split_ip_to_bytes_method3(upload_ip);
	for (uint8_t i = 0; i < 4; i++)
		cmd[i] = iplist[i];

	memcpy(cmd + 4, &upload_port, 2);
	std::string send_buf = BaseAPI::generate_cmd2(3, 6, cmd);
	
	int recv_len;
	char recv_buf[1024] = {0};
	ret = CommunicationAPI::udp_talk_pack(lidar->udp_cmd_fd, lidar->lidar_ip.c_str(), lidar->lidar_port, send_buf.size(), send_buf.c_str(), recv_len, recv_buf);
	if (ret)
	{
		
		printf("SetLidarUploadNetWork:%s\n",recv_buf);
		return ret;
	}
	return false;
}

bool PaceCatLidarSDK::SetTimeStampSync(int ID)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return OFFLINE;

	KeepAlive alive;
	struct timeval tv;
	SystemAPI::GetTimeStamp(&tv, true);
	alive.world_clock.second = tv.tv_sec;
	alive.world_clock.nano_second = tv.tv_usec * 1000;

	std::string send_buf = BaseAPI::generate_cmd(0x00, rand(), 0x04, sizeof(KeepAlive), &alive);
	int recv_len;
	char recv_buf[1024] = {0};
	bool ret = CommunicationAPI::udp_talk_pack(lidar->udp_cmd_fd, lidar->lidar_ip.c_str(), lidar->lidar_port, send_buf.size(), send_buf.c_str(), recv_len, recv_buf);
	if (ret)
	{
		uint32_t code;
		memcpy(&code, recv_buf, sizeof(code));
		return code == 0 ? true : false;
	}
	return false;
}

bool PaceCatLidarSDK::SetWorking(int ID, bool isrun)
{
	RunConfig *lidar = getConfig(ID);
	if (lidar == nullptr)
		return false;

	std::string send_buf = BaseAPI::generate_cmd(0x00, rand(), 0x05, 1, &isrun);
	int recv_len;
	char recv_buf[1024] = {0};
	bool ret = CommunicationAPI::udp_talk_pack(lidar->udp_cmd_fd, lidar->lidar_ip.c_str(), lidar->lidar_port, send_buf.size(), send_buf.c_str(), recv_len, recv_buf, 10, 50000);
	if (ret)
	{
		uint8_t code;
		memcpy(&code, recv_buf, sizeof(code));
		return code == 0 ? true : false;
	}
	return false;
}
int PaceCatLidarSDK::QueryIDByIp(std::string ip)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->lidar_ip == ip && m_lidars.at(i)->run_state != QUIT)
		{
			return m_lidars.at(i)->ID;
		}
	}
	return -1;
}

RunConfig *PaceCatLidarSDK::getConfig(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			return m_lidars[i];
		}
	}
	return nullptr;
}

void PaceCatLidarSDK::UDPThreadRecv(int id)
{

	RunConfig *cfg = getConfig(id);
	if (!cfg)
		return;

	struct timeval to = {0, 10};
	while (cfg->run_state != QUIT)
	{
		if (cfg->run_state == OFFLINE)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		uint8_t recv_buf[4096];
		sockaddr_in addr;
		socklen_t sz = sizeof(addr);

		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(cfg->udp_data_fd, &fds);
		int ret = select(cfg->udp_data_fd + 1, &fds, NULL, NULL, &to);
		if (ret < 0)
		{
			cfg->run_state = QUIT;
			break;
		}
		else if (ret == 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}
		if (FD_ISSET(cfg->udp_data_fd, &fds))
		{
			int dw = recvfrom(cfg->udp_data_fd, (char *)&recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr, &sz);
			if (dw > 0 && (strcmp((char *)inet_ntoa(addr.sin_addr), cfg->lidar_ip.c_str()) == 0))
			{
				cfg->data_queue.enqueue(std::string((char *)recv_buf, dw));
			}
		}
	}
	std::string err = "thread sub end";
	WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
}

void PaceCatLidarSDK::UDPThreadParse(int id)
{
	RunConfig *cfg = getConfig(id);
	onePoi *oneFramePoi;
	int LH_HEIGHT = 150;
	int LH_WIDTH = 360;
	// judge   single echo  or double  echo;
	int upload_type = getbit(cfg->dev_cfg.upload_point_clound_en, 0) * 1 + getbit(cfg->dev_cfg.upload_point_clound_en, 1) * 2 + getbit(cfg->dev_cfg.upload_point_clound_en, 2) * 4;
	if (upload_type == 1)
		LH_HEIGHT = 150;
	else if (upload_type == 2)
		LH_HEIGHT = 75;

	oneFramePoi = new onePoi[LH_HEIGHT * LH_WIDTH];
	int lastFrameidx = 0;
	fs_lidar_imu_t change_imu;

	while (cfg->run_state != QUIT)
	{
		std::string recv_data;
		bool ret = cfg->data_queue.try_dequeue(recv_data);
		if (!ret)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}
		char *recv_buf = (char *)recv_data.c_str();
		soc_head_t *head = (soc_head_t *)recv_buf;

		if ((unsigned char)recv_buf[0] == 0x41 && (unsigned char)recv_buf[1] == 0x4d)
		{
			PROTOCOL_DATA_PACK_ALARM_ST *data_pack_alarm = (PROTOCOL_DATA_PACK_ALARM_ST *)(recv_buf + 20);
			std::string errorevent = GetErrorEvent(&data_pack_alarm->events);
			if (!errorevent.empty())
			{
				std::string err = "time: " + SystemAPI::getCurrentTime() + " alarm info: " + errorevent;
				WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
			}
		}
		if ((unsigned char)recv_buf[0] != 0x5A || (unsigned char)recv_buf[1] != 0xA5)
			continue;
		if (head->data_type == 0x01 || head->data_type == 0x0E)
		{
			soc_protocol_data_t *pro_data = (soc_protocol_data_t *)(recv_buf + sizeof(soc_head_t));
			uint16_t start_row = pro_data->row;
			uint16_t start_col = pro_data->col;
			uint32_t row = 0;
			uint32_t col = 0;
			uint16_t pixelcnt = pro_data->dot_num * pro_data->echo_num;
			uint16_t one_pixel_len = sizeof(soc_dis_pt_data_t);
			int head_size = sizeof(soc_head_t) + sizeof(soc_protocol_data_t);
			if (lastFrameidx != 0)
			{
				if ((pro_data->data_sequence - lastFrameidx != 1) && (pro_data->data_sequence != 1))
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " drop package last index: " + std::to_string(lastFrameidx) + " now index:" + std::to_string(pro_data->data_sequence);
					WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
				}
			}
			lastFrameidx = pro_data->data_sequence;
			for (int j = 0; j < pixelcnt; j++)
			{
				if (head->data_type == 0x01)
				{
					row = start_row + j / 4;
					col = start_col + j % 4;
				}
				else if (head->data_type == 0x0E)
				{
					row = start_row / 2 + j / 4;
					col = start_col + j % 4;
				}

				onePoi ptInfo;
				memset(&ptInfo, 0, sizeof(onePoi));
				ptInfo.row_pos = row;
				ptInfo.col_pos = col;
				ptInfo.timestamp = pro_data->timestamp;
				ptInfo.nbVal = pro_data->time_type;
				soc_dis_pt_data_t *t = (soc_dis_pt_data_t *)(recv_buf + head_size + j * one_pixel_len);
				ptInfo.pt3d.x = (float)(t->x / 10.0f) * 2 / 100;
				ptInfo.pt3d.y = (float)(t->y / 10.0f) * 2 / 100;
				ptInfo.pt3d.z = (float)(t->z / 10.0f) * 2 / 100;
				ptInfo.distance = t->distance;
				ptInfo.reflectivity = t->reflect;
				memcpy(oneFramePoi + (row * LH_WIDTH + col), &ptInfo, sizeof(onePoi));
			}
			if (pro_data->frame_flag == 2)
			{
				PaceCatLidarSDK::getInstance()->WritePointCloud(cfg->ID, 0, oneFramePoi, LH_HEIGHT * LH_WIDTH);
				memset(oneFramePoi, 0, sizeof(onePoi) * LH_HEIGHT * LH_WIDTH);
			}
		}
		else if (head->data_type == 4)
		{
			fs_lidar_imu_header_t *imu_header = (fs_lidar_imu_header_t *)(recv_buf + sizeof(soc_head_t));
			change_imu.version = imu_header->version;
			change_imu.timestamp = imu_header->timestamp;
			if (imu_header->version == 2)
			{
				fs_lidar_imu_v2 *imu_t = (fs_lidar_imu_v2 *)(recv_buf + sizeof(soc_head_t) + sizeof(fs_lidar_imu_header_t));
				memcpy(&change_imu.gyro_x, imu_t, sizeof(fs_lidar_imu_v2));
			}
			else if (imu_header->version == 3)
			{

				fs_lidar_imu_v3 *imu_t = (fs_lidar_imu_v3 *)(recv_buf + sizeof(soc_head_t) + sizeof(fs_lidar_imu_header_t));
				// printf("%f %f %f\n",imu_t->pitch,imu_t->roll,imu_t->yaw);
				memcpy(&change_imu.pitch, imu_t, sizeof(fs_lidar_imu_v3));
			}
			WriteImuData(cfg->ID, 0, &change_imu);
		}
		else if (head->data_type == 8)
		{
			// 报警信息
		}
		else
		{
			const uint8_t *ptr = (uint8_t *)&recv_buf;
			printf("%ld : %02x %02x %02x %02x %02x %02x %02x %02x\n",
				   recv_data.size(),
				   ptr[0], ptr[1], ptr[2], ptr[3],
				   ptr[4], ptr[5], ptr[6], ptr[7]);
		}
	}
	delete[] oneFramePoi;
	std::string err = "thread parse end";
	WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
}

void PaceCatLidarSDK::HeartThreadProc(HeartInfo &heartinfo)
{
#ifdef _WIN32
	WSADATA wsda; //   Structure   to   store   info
	WSAStartup(MAKEWORD(2, 2), &wsda);
#endif
	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	int yes = 1;
	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *)&yes, sizeof(yes)) < 0)
	{
		heartinfo.value = "socket init error";
		heartinfo.code = SystemAPI::getLastError();
		SystemAPI::closefd(sock, true);
		return;
	}

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(HEARTPORT);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int iResult = ::bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (iResult != 0)
	{
		heartinfo.value = "bind port failed";
		heartinfo.code = SystemAPI::getLastError();
		SystemAPI::closefd(sock, true);
		return;
	}

	struct ip_mreq mreq;
	mreq.imr_multiaddr.s_addr = inet_addr("225.225.225.225");
	mreq.imr_interface.s_addr = get_interface_ip(heartinfo.adapter.c_str());
	if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq)) < 0)
	{
		heartinfo.value = "socket init error";
		heartinfo.code = SystemAPI::getLastError();
		SystemAPI::closefd(sock, true);
		return;
	}

	struct timeval tv;
	SystemAPI::GetTimeStamp(&tv, false);
	time_t tto = tv.tv_sec + 1;
	socklen_t sz = sizeof(addr);

	while (heartinfo.isrun)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(sock, &fds);
		struct timeval to = {0, 10};
		int ret = select(sock + 1, &fds, NULL, NULL, &to);
		if (ret > 0)
		{
			char raw[4096];
			int dw = recvfrom(sock, raw, sizeof(raw), 0, (struct sockaddr *)&addr, &sz);
			// printf("%s %d %d %d \n",__FUNCTION__,__LINE__,dw,sizeof(DevHeart));
			if (dw == sizeof(DevHeart))
			{
				DevHeart *devheart = (DevHeart *)raw;
				std::string sn = BaseAPI::stringfilter((char *)devheart->dev_sn, 20);
				char tmp_ip[16];
				sprintf(tmp_ip, "%d.%d.%d.%d", devheart->ip[0], devheart->ip[1], devheart->ip[2], devheart->ip[3]);
				std::string ip = BaseAPI::stringfilter(tmp_ip, 16);
				int id = QueryIDByIp(ip);
				bool isexist = false;
				for (unsigned int i = 0; i < heartinfo.lidars.size(); i++)
				{
					if (sn == heartinfo.lidars[i].sn)
					{
						heartinfo.lidars[i].timestamp = devheart->timestamp.second * 1000 + devheart->timestamp.nano_second / 1000000;
						isexist = true;
						heartinfo.lidars[i].flag = true;
						if (!heartinfo.lidars[i].state)
						{
							heartinfo.lidars[i].state = true;

							std::string result = sn + " " + ip + "  online";
							WriteLogData(id, 0, (char *)result.c_str(), result.size());
						}
						break;
					}
					// exist  two more lidars with same ip
					if (ip == heartinfo.lidars[i].ip)
					{
						heartinfo.value = "multiple lidars with the same ip";
						heartinfo.code = -1;
						// heartinfo.isrun=false;
						break;
					}
				}
				if (!isexist)
				{
					ConnectInfo info;
					info.ip = ip;
					info.sn = sn;
					info.port = devheart->remote_port;
					info.timestamp = devheart->timestamp.second * 1000 + devheart->timestamp.nano_second / 1000000;
					info.state = true;
					info.flag = true;
					heartinfo.lidars.push_back(info);
					std::string result = sn + " " + ip + "  online";
					WriteLogData(id, 0, (char *)result.c_str(), result.size());
				}
			}
		}

		// check is outtime
		SystemAPI::GetTimeStamp(&tv, false);
		if (tv.tv_sec > tto)
		{
			for (unsigned int i = 0; i < heartinfo.lidars.size(); i++)
			{
				if (heartinfo.lidars[i].state == ONLINE && !heartinfo.lidars[i].flag)
				{
					int id = QueryIDByIp(heartinfo.lidars[i].ip);
					heartinfo.lidars[i].state = false;
					std::string result = heartinfo.lidars[i].sn + " " + heartinfo.lidars[i].ip + "  offline";
					WriteLogData(id, 0, (char *)result.c_str(), result.size());
				}
				heartinfo.lidars[i].flag = false;
			}
			tto = tv.tv_sec + 1;
		}
	}
	SystemAPI::closefd(sock, true);
	std::string err = "thread heart end";
	printf("%s\n", err.c_str());
}

std::string PaceCatLidarSDK::GetErrorEvent(PROCOTOL_ALARM_EVENTS_ST *syseventlog)
{
	std::stringstream eventlog_ss;
	if (syseventlog->tx_tem_h)
		eventlog_ss << " send high temperature";
	if (syseventlog->rx_tem_h)
		eventlog_ss << " recv high temperature";
	if (syseventlog->tx_vol_h)
		eventlog_ss << " send high voltage";
	if (syseventlog->tx_vol_l)
		eventlog_ss << " send low voltage";
	if (syseventlog->rx_vol_h)
		eventlog_ss << " recv high voltage";
	if (syseventlog->rx_vol_l)
		eventlog_ss << " recv low voltage";
	if (syseventlog->no_cali_table)
		eventlog_ss << " no calib table";
	if (syseventlog->data_is_zero)
		eventlog_ss << " data is zero";
	if (syseventlog->no_imu_data)
		eventlog_ss << " no imu data";

	return eventlog_ss.str();
}

in_addr_t PaceCatLidarSDK::get_interface_ip(const char *ifname)
{
	if (!ifname || strlen(ifname) == 0)
	{
		return INADDR_NONE;
	}

#ifdef _WIN32
	return 0;
#else
	// Linux/Unix 实现 (保持不变)
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0)
	{
		perror("socket");
		return INADDR_NONE;
	}

	struct ifreq ifr;
	strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';

	if (ioctl(fd, SIOCGIFADDR, &ifr) < 0)
	{
		perror("ioctl(SIOCGIFADDR)");
		close(fd);
		return INADDR_NONE;
	}

	close(fd);
	return ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr;
#endif
}