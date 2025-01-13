#ifndef TCP_TRANS_H
#define TCP_TRANS_H

#include <string>

/*
 * ������
 */
enum
{
	FS_EC_OK = 0,
	FS_EC_INIT_SOCK_FAILD,				
	FS_EC_LINK_CTRLSRV_FAILD,			
	FS_EC_SEND_FAILD,					
	FS_EC_LINK_ERROR,					
};

/*
 * ����ָ��
 */
enum
{
	FS_HAL_CMD_UNDEFINED = 0x00,			
	FS_HAL_CMD_GET_MAX_PKG_SIZE = 0x01,		
	FS_HAL_CMD_SET_MAX_PKG_SIZE = 0x02,		
	FS_LIDAR_CMD_SET_DEV_IP_PORT = 0x201, 
	FS_LIDAR_CMD_GET_DEV_IP_PORT = 0x202, 
};

#pragma pack(push,1)
#define LOG_LOCAL_PORT		6402
#define LOG_TARGET_PORT		6401
#define CTRL_LOCAL_PORT		6502
#define CTRL_TARGET_PORT	6501

#define PRT_MIAN_VER		0x01
#define PRT_CHILD_VER		0x01
#define PRT_HEAD_LEN		sizeof(PrtHead)
#define CTRL_CMD_HEAD_LEN	sizeof(CtrlCmdHead)

typedef struct
{
	uint8_t head[2];		
	uint8_t main_ver;		
	uint8_t type;			
	uint16_t reserve;		
}PrtHead;

typedef struct
{
	uint8_t child_ver;		
	uint8_t head[2];		
	uint16_t cmd;			
	uint16_t len;			
	uint8_t reserve[6];		
}CtrlCmdHead;

typedef struct
{
	PrtHead prt_head;
	CtrlCmdHead ctrl_cmd_head;
	uint8_t head_crc;		
	uint8_t cmd_crc;		
	std::string data;		
}CtrlPrtDataPkg;

#pragma pack(pop)
// ����crc8
uint8_t fs_crc_8_l(uint8_t* data, uint32_t dlen, uint8_t init = 0x00);

CtrlPrtDataPkg pack_ctrl_prt_data_pkg(uint16_t cmd, uint16_t len, uint8_t* data);
int link_to_device_tcp(const std::string target_ip, uint32_t link_timeout);

void break_link_tcp();

int send_cmd_tcp(uint16_t cmd_type, uint8_t* in_data, uint16_t data_len, std::string& out_data, uint32_t ms_timeout = 1000, uint16_t resend_cnt = 2);

#endif // !TCP_TRANS_H
