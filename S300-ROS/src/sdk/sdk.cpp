// M300_SDK.cpp: 定义应用程序的入口点。
//
#include "sdk.h"
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
}
bool PaceCatLidarSDK::SetPointCloudCallback(int ID, LidarCloudPointCallback cb, void *client_data)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	lidar->cb_cloudpoint = cb;
	lidar->cloudpoint = client_data;
	return true;
}
bool PaceCatLidarSDK::SetImuDataCallback(int ID, LidarImuDataCallback cb, void *client_data)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	lidar->cb_imudata = cb;
	lidar->imudata = (char *)client_data;
	return true;
}
bool PaceCatLidarSDK::SetLogDataCallback(int ID, LidarLogDataCallback cb, void *client_data)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	lidar->cb_logdata = cb;
	lidar->logdata = (char *)client_data;
	return true;
}

void PaceCatLidarSDK::WritePointCloud(int ID, const uint8_t dev_type, onePoi *data)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return;
	if (lidar->cb_cloudpoint != nullptr)
		lidar->cb_cloudpoint(ID, dev_type, data, lidar->cloudpoint);
}

void PaceCatLidarSDK::WriteImuData(int ID, const uint8_t dev_type, fs_lidar_imu_t *data)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return;
	if (lidar->cb_imudata != nullptr)
		lidar->cb_imudata(ID, dev_type, data, lidar->imudata);
}

void PaceCatLidarSDK::WriteLogData(int ID, const uint8_t dev_type, char *data, int len)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return;
	if (lidar->cb_logdata != nullptr)
		lidar->cb_logdata(ID, dev_type, data, len);
}

void PaceCatLidarSDK::Init()
{
}

void PaceCatLidarSDK::Uninit()
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		m_lidars.at(i)->run_state = QUIT;
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
	m_lidars.push_back(cfg);
	return cfg->ID;
}

bool PaceCatLidarSDK::ConnectLidar(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	lidar->thread_subData = std::thread(UDPThreadProc, ID);
	lidar->thread_subData.detach();
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

bool PaceCatLidarSDK::QueryBaseInfo(int ID, std::string lidarip, fs_ipport_t &info)
{
	bool result = false;
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return result;

	if (!is_ip(lidarip))
	{
		std::string err = "time: " + SystemAPI::getCurrentTime() + " lidar ip set is illegal  " + lidarip;
		PaceCatLidarSDK::getInstance()->WriteLogData(lidar->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		return result;
	}

	int errCode = link_to_device_tcp(lidar->lidar_ip, 2000); // 2000  连接2秒连不上，为超时
	if (errCode != 0)
	{
		return result;
	}

	FsTcpTransControl *tcp = FsTcpTransControl::getInstance();
	result = tcp->get_ipport(&info);
	break_link_tcp();
	return result;
}
bool PaceCatLidarSDK::QueryVersion(int ID, VersionInfo &info)
{
	return false;
}
int PaceCatLidarSDK::QueryDeviceState(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return OFFLINE;

	// add

	int errCode = link_to_device_tcp(lidar->lidar_ip, 2000); // 2000  连接2秒连不上，为超时
	if (errCode != 0)
	{
		std::string err = "time: " + SystemAPI::getCurrentTime() + " device connect   fail  " + lidar->lidar_ip;
		PaceCatLidarSDK::getInstance()->WriteLogData(lidar->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		return OFFLINE;
	}

	return ONLINE;
}

bool PaceCatLidarSDK::SetLidarNetWork(int ID, std::string lidar_ip, uint16_t lidar_port, std::string host_ip, uint16_t host_port)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	if (!is_ip(lidar_ip))
	{
		std::string err = "time: " + SystemAPI::getCurrentTime() + " lidar ip set is illegal  " + lidar_ip;
		PaceCatLidarSDK::getInstance()->WriteLogData(lidar->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		return false;
	}
	if (!is_ip(host_ip))
	{
		std::string err = "time: " + SystemAPI::getCurrentTime() + " upload ip set is illegal  " + host_ip;
		PaceCatLidarSDK::getInstance()->WriteLogData(lidar->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		return false;
	}

	int errCode = link_to_device_tcp(lidar->lidar_ip, 2000); // 2000  连接2秒连不上，为超时
	if (errCode != 0)
	{
		std::string err = "time: " + SystemAPI::getCurrentTime() + " device connect   fail  " + lidar->lidar_ip;
		PaceCatLidarSDK::getInstance()->WriteLogData(lidar->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		return false;
	}

	FsTcpTransControl *tcp = FsTcpTransControl::getInstance();
	fs_ipport_t ipport_t = {0};
	ipport_t.lidar_ip = inet_addr(lidar_ip.data());
	ipport_t.host_ip = inet_addr(host_ip.data());
	ipport_t.lidar_port = lidar_port;
	ipport_t.host_port = host_port;
	bool set_ret = tcp->set_ipport(ipport_t);
	break_link_tcp();
	return set_ret;
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
		if (m_lidars.at(i)->ID == ID)
		{
			return m_lidars[i];
		}
	}
	return nullptr;
}

void UDPThreadProc(int id)
{
	RunConfig *cfg = PaceCatLidarSDK::getInstance()->getConfig(id);

	int fd = SystemAPI::open_socket_port(cfg->listen_port, false);
	if (fd <= 0)
	{
		std::string err = "listen port open failed";
		PaceCatLidarSDK::getInstance()->WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		return;
	}
	int tcp_cmd_fd = SystemAPI::open_tcp_socket_port(cfg->lidar_ip.c_str(), cfg->lidar_port);
	if (tcp_cmd_fd <= 0)
	{
		std::string err = "tcp cmd port open failed";
		PaceCatLidarSDK::getInstance()->WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		return;
	}
	
	onePoi *m_oneFramePoi = new onePoi[HEIGHT * WIDTH];
	
	fs_lidar_imu_t change_imu;
	
	uint16_t wait_idx = 0;
	uint8_t PaceCat_idx = 0;
	uint16_t nlen = 0;
	int lastFrameidx = 0;
	struct timeval tv;
	SystemAPI::GetTimeStamp(&tv, NULL);
	
	time_t tto = tv.tv_sec + 1;
	
	while (cfg->run_state != QUIT)
	{
		if (cfg->run_state == OFFLINE)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		SystemAPI::GetTimeStamp(&tv, NULL);
		if (tv.tv_sec > tto)
		{
			KeepAlive live;
			live.world_clock.second = tv.tv_sec;
			live.world_clock.nano_second = tv.tv_usec * 1000;
			//CommunicationAPI::send_cmd_udp(tcp_cmd_fd, cfg->lidar_ip.c_str(), cfg->lidar_port, 0x00, rand(), 0x04, 0x18, &live);
			tto = tv.tv_sec + 1;
		}

		uint8_t recv_buf[4096];
		sockaddr_in addr;
		socklen_t sz = sizeof(addr);
		int dw = recvfrom(fd, (char *)&recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr, &sz);
		if (dw > 0 && (strcmp((char *)inet_ntoa(addr.sin_addr), cfg->lidar_ip.c_str()) == 0))
		{

			soc_head_t *head = (soc_head_t *)recv_buf;
			if (head->data_type == 0x01 || head->data_type == 0x0C || head->data_type == 0x0D || head->data_type == 0x0E)
			{
				soc_protocol_data_t *pro_data = (soc_protocol_data_t *)(recv_buf + sizeof(soc_head_t));
				uint16_t start_row = pro_data->row;
				uint16_t start_col = pro_data->col;
				uint32_t row = 0;
				uint32_t col = 0;
				uint16_t pixelcnt = pro_data->dot_num * pro_data->echo_num;
				uint16_t one_pixel_len = sizeof(soc_dis_pt_data_t);
				int head_size = sizeof(soc_head_t) + sizeof(soc_protocol_data_t);

				// std::string err = "row:"+std::to_string(start_row)+" col:"+std::to_string(start_col);
				// PaceCatLidarSDK::getInstance()->WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
				// MYLOG << pro_data->row << pro_data->col << pro_data->echo_num << pro_data->dot_num << pro_data->frame_flag << pro_data->data_sequence;

				if ((pro_data->data_sequence - lastFrameidx != 1) && (pro_data->data_sequence != 1))
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " drop package last index: " + std::to_string(lastFrameidx) + " now index:" + std::to_string(pro_data->data_sequence);
					PaceCatLidarSDK::getInstance()->WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
					lastFrameidx = pro_data->data_sequence;
					continue;
				}
				lastFrameidx = pro_data->data_sequence;
				for (int j = 0; j < pixelcnt; j++)
				{
					if (head->data_type == 0x01)
					{
						row = start_row + j / 4;
						col = start_col + j % 4;
						if (row > HEIGHT - 1)
							break;
						if (col > WIDTH - 1)
							break;
					}
					else if (head->data_type == 0x0C)
					{
						row = start_row / 2 + j / 2;
						col = start_col / 2 + j % 2;
						if (row > HEIGHT / 2 - 1)
							break;
						if (col > WIDTH / 2 - 1)
							break;
					}
					else if (head->data_type == 0x0D)
					{
						//row = start_row + (j / 8) * 2;
						//col = start_col + (j / 2) % 4;
						row = start_row + j / 4;
						col = start_col + j % 4;
						if (row > HEIGHT - 1)
							break;
						if (col > WIDTH - 1)
							break;
					}
					else if (head->data_type == 0x0E)
					{
						row = start_row + (j / 4) * 2;
						col = start_col + j % 4;
						if (row > HEIGHT - 1)
							break;
						if (col > WIDTH - 1)
							break;
					}
					
					
					onePoi ptInfo;
					memset(&ptInfo, 0, sizeof(onePoi));
					ptInfo.row_pos = row;
					ptInfo.col_pos = col;
					ptInfo.timestamp = pro_data->timestamp;
					ptInfo.nbVal = pro_data->time_type;
					soc_dis_pt_data_t *t = (soc_dis_pt_data_t *)(recv_buf + head_size + j * one_pixel_len);
					ptInfo.pt3d.x = (double)(t->x / 10.0f) * 2 / 100;
					ptInfo.pt3d.y = (double)(t->y / 10.0f) * 2 / 100;
					ptInfo.pt3d.z = (double)(t->z / 10.0f) * 2 / 100;
					ptInfo.distance = t->distance;
					ptInfo.reflectivity = t->reflect;
					memcpy(m_oneFramePoi + (row * WIDTH + col), &ptInfo, sizeof(onePoi));
				}
				if (pro_data->frame_flag == 2)
				{
					PaceCatLidarSDK::getInstance()->WritePointCloud(cfg->ID, 0, m_oneFramePoi);
					memset(m_oneFramePoi, 0, sizeof(onePoi) * HEIGHT * WIDTH);
				}
			}
			else if (head->data_type == 4)
			{
				fs_lidar_imu_t *imu_t = (fs_lidar_imu_t *)(recv_buf + sizeof(soc_head_t));
				imu_t->timestamp = imu_t->timestamp;
				memcpy(&change_imu, imu_t, sizeof(fs_lidar_imu_t));
				PaceCatLidarSDK::getInstance()->WriteImuData(cfg->ID, 0, &change_imu);
			}
			else if (head->data_type == 8)
			{
				// 报警信息
			}
			else
			{
				const uint8_t *ptr = (uint8_t *)&recv_buf;
				printf("%d : %02x %02x %02x %02x %02x %02x %02x %02x\n",
					   dw,
					   ptr[0], ptr[1], ptr[2], ptr[3],
					   ptr[4], ptr[5], ptr[6], ptr[7]);
			}
		}
		switch (cfg->action)
		{
		case NONE:
		case FINISH:
		{
			break;
		}
		}
	}
	SystemAPI::closefd(fd, true);

	std::string err = "thread sub end";
	PaceCatLidarSDK::getInstance()->WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
}
