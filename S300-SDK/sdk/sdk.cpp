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
	heartinfo.isrun=false;
	for(int i=0;i<m_lidars.size();i++)
	{
		m_lidars[i]->run_state==OFFLINE;
		SystemAPI::closefd(m_lidars[i]->udp_data_fd, true);
		SystemAPI::closefd(m_lidars[i]->tcp_cmd_fd, true);
	}

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
	heartinfo.isrun = true;
	std::thread tt = std::thread(&PaceCatLidarSDK::HeartThreadProc, PaceCatLidarSDK::getInstance());
	tt.detach();
}

void PaceCatLidarSDK::Uninit()
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		m_lidars.at(i)->run_state = QUIT;
		SystemAPI::closefd(m_lidars[i]->tcp_cmd_fd, true);
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
	lidar->udp_data_fd = SystemAPI::open_socket_port(lidar->listen_port, false);
	if (lidar->udp_data_fd <= 0)
	{
		std::string err = "listen port open failed";
		WriteLogData(lidar->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		return false;
	}
	lidar->tcp_cmd_fd = SystemAPI::open_tcp_socket_port(lidar->lidar_ip.c_str(), lidar->lidar_port);
	if (lidar->tcp_cmd_fd <= 0)
	{
		std::string err = "tcp cmd port open failed";
		WriteLogData(lidar->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		SystemAPI::closefd(lidar->udp_data_fd, true);
		return false;
	}

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

bool PaceCatLidarSDK::QueryBaseInfo(int ID, std::string lidarip, fs_ipport_t &info)
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

	char str = 0x00;
	std::string sendbuf = BaseAPI::generate_cmd(0x00, rand(), 0x01, 1, &str);

	std::string recvbuf;
	if (TalkWithTCP(lidar->tcp_cmd_fd, 2, sendbuf, recvbuf))
	{
		DEV_CFG_ST *cfg = (DEV_CFG_ST *)recvbuf.c_str();
		memcpy(&info.host_ip, cfg->upload_ip, 4);
		info.host_port = cfg->upload_port;
		memcpy(&info.lidar_ip, cfg->local_ip, 4);
		info.lidar_port = cfg->local_port;
		return true;
	}
	return false;
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

	std::string ip = lidar->lidar_ip;

	for (unsigned int i = 0; i < heartinfo.lidars.size(); i++)
	{
		std::string tmpip = heartinfo.lidars[i].ip;
		if (ip == tmpip)
		{
			return heartinfo.lidars[i].state;
		}
	}
	return OFFLINE;
}

bool PaceCatLidarSDK::SetLidarNetWork(int ID, std::string lidar_ip, uint16_t lidar_port, std::string host_ip, uint16_t host_port)
{
	return false;
}

bool PaceCatLidarSDK::SetYawAngle(int ID, int yaw)
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

	char cmd[4] = {0};
	memcpy(cmd, &yaw, 4);

	std::string sendbuf = BaseAPI::generate_cmd2(15, 4, cmd);
	std::string recvbuf;
	return TalkWithTCP(lidar->tcp_cmd_fd, 1, sendbuf, recvbuf);
}
bool PaceCatLidarSDK::SetTimeStampSync(int ID)
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

	return SetTimeSync(lidar->tcp_cmd_fd);
}

bool PaceCatLidarSDK::TalkWithTCP(int tcp_cmd_fd, int waittime, const std::string sendbuf, std::string &recvbuf)
{
	int result = send(tcp_cmd_fd, sendbuf.c_str(), sendbuf.size(), 0);
	if (result == -1)
	{
		if (errno == ECONNRESET || errno == EPIPE)
		{
			return false;
		}
	}
	unsigned char recv_cmd_buffer[1024] = {0}; // 数据接收缓存
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(tcp_cmd_fd, &fds);
	struct timeval to = {10, 0};
	int ret = select(tcp_cmd_fd + 1, &fds, NULL, NULL, &to);
	if (ret <= 0)
	{
		return false;
	}
	if (FD_ISSET(tcp_cmd_fd, &fds))
	{
		int tcp_len = recv(tcp_cmd_fd, recv_cmd_buffer, sizeof(recv_cmd_buffer), 0);
		if (tcp_len)
		{
			if (recv_cmd_buffer[0] == 0x4c && recv_cmd_buffer[1] == 0x48)
			{
				CmdHeader hdr;
				memcpy(&hdr, &recv_cmd_buffer[0], sizeof(CmdHeader));
				int cmd_len = sizeof(CmdHeader) + hdr.len + 1;
				if (tcp_len == cmd_len)
				{
					if (hdr.cmd == 0x8000)
						return true;
					else if (hdr.cmd == 0x8001)
					{
						DEV_CFG_ST *cfg = (DEV_CFG_ST *)(recv_cmd_buffer + sizeof(CmdHeader));
						recvbuf = std::string((char *)cfg, sizeof(DEV_CFG_ST));
						return true;
					}
					else if (hdr.cmd == 0x8002)
					{
						uint32_t code;
						memcpy(&code, recv_cmd_buffer + sizeof(CmdHeader), sizeof(code));
						if (code == 0)
							return true;
						else
							return false;
					}
					else if (hdr.cmd == 0x8004)
					{
						uint32_t code;
						memcpy(&code, recv_cmd_buffer + sizeof(CmdHeader), sizeof(code));
						if (code == 0)
							return true;
						else
							return false;
					}
				}
			}
			else
			{
				return false;
			}
		}
	}
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
bool PaceCatLidarSDK::SetTimeSync(int fd)
{
	KeepAlive alive;
	struct timeval tv;
	SystemAPI::GetTimeStamp(&tv, true);
	alive.world_clock.second = tv.tv_sec;
	alive.world_clock.nano_second = tv.tv_usec * 1000;

	std::string sendbuf = BaseAPI::generate_cmd(0x00, rand(), 0x04, sizeof(KeepAlive), &alive);
	std::string recvbuf;
	return TalkWithTCP(fd, 1, sendbuf, recvbuf);
}
void PaceCatLidarSDK::UDPThreadRecv(int id)
{
	RunConfig *cfg = getConfig(id);
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
		int ret = select(cfg->udp_data_fd+1, &fds, NULL, NULL, &to);
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
	int lastFrameidx = 0;
	onePoi *oneFramePoi = new onePoi[HEIGHT * WIDTH];
	fs_lidar_imu_t change_imu;
	RunConfig *cfg = getConfig(id);
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
					// row = start_row + (j / 8) * 2;
					// col = start_col + (j / 2) % 4;
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
				memcpy(oneFramePoi + (row * WIDTH + col), &ptInfo, sizeof(onePoi));
			}
			if (pro_data->frame_flag == 2)
			{
				PaceCatLidarSDK::getInstance()->WritePointCloud(cfg->ID, 0, oneFramePoi);
				memset(oneFramePoi, 0, sizeof(onePoi) * HEIGHT * WIDTH);
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
		else if (recv_buf[0] == 0x41 && recv_buf[1] == 0x4d)
		{
		}
		else
		{
			const uint8_t *ptr = (uint8_t *)&recv_buf;
			printf("%d : %02x %02x %02x %02x %02x %02x %02x %02x\n",
				   recv_data.size(),
				   ptr[0], ptr[1], ptr[2], ptr[3],
				   ptr[4], ptr[5], ptr[6], ptr[7]);
		}
	}
	delete[] oneFramePoi;
	std::string err = "thread parse end";
	WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
}

void PaceCatLidarSDK::HeartThreadProc()
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
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
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
	printf("%s\n",err.c_str());
}
