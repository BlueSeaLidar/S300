#include "TcpTrans.h"
#include <chrono>
#include <thread>
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#if (_WINDLL || _WINDOWS || _WIN32)

#include <stdint.h>
#include <io.h>
#include <direct.h>
#include <time.h>
#include <WinSock2.h>
// #include <WS2tcpip.h>

#else

#include <sys/socket.h>
#include <unistd.h>
#include <sched.h>
#include <netinet/in.h> //struct sockaddr_in 结构
#include <arpa/inet.h>  //字节序转换接口文件
#include <sys/socket.h> //socket接口文件
#include <sys/stat.h>
#include <string.h>
#include <fcntl.h>
#endif
#define MAX_TRANS_PKG_LEN (1024 * 8)
#define CRC8_POLYNOMIAL 0x31

/**************** 一些全局变量 **************************/
int sock_ctrl = -1;            // socket控制套接字
uint32_t g_max_pkg_len = 1024; // 收/发最大包长
bool g_link_net = false;       // 保存网络连接状态
/********************************************************/

/**************** 一些内部函数 **************************/
bool UnblockConnect(int sock, struct sockaddr_in *to_addr, uint32_t link_timeout);
uint32_t get_max_pkg_size();
bool set_max_pkg_size(uint32_t maxPkgSize);
/********************************************************/

uint8_t fs_crc_8_l(uint8_t *data, uint32_t dlen, uint8_t init)
{
    uint8_t *buf = (uint8_t *)data;
    unsigned char usResult = init;
    unsigned int i = 0;
    unsigned int j = 0;
    unsigned char usVal;

    for (i = 0; i < dlen; i++)
    {
        usVal = buf[i];

        for (j = 0; j < 8; j++)
        {
            if (usResult & 0x80)
            {
                usResult = (usResult << 1) | 0x01;
                usResult ^= 0x30;

                if (usVal & 0x80)
                {
                    usResult ^= CRC8_POLYNOMIAL;
                }
            }
            else
            {
                usResult <<= 1;

                if (usVal & 0x80)
                {
                    usResult ^= CRC8_POLYNOMIAL;
                }
            }

            usVal <<= 1;
        }
    }
    return usResult;
}

CtrlPrtDataPkg pack_ctrl_prt_data_pkg(uint16_t cmd, uint16_t len, uint8_t *data)
{
    PrtHead prtHead = {{0x5A, 0xA5}, PRT_MIAN_VER, 0x06};
    CtrlCmdHead ccHead = {PRT_CHILD_VER, {0x65, 0x9A}, cmd, len};
    CtrlPrtDataPkg cpdp = {prtHead, ccHead};
    cpdp.head_crc = fs_crc_8_l((uint8_t *)&cpdp, PRT_HEAD_LEN + CTRL_CMD_HEAD_LEN, 0x00);
    cpdp.cmd_crc = fs_crc_8_l(data, len, 0x00);
    cpdp.data.assign((char *)data, len);

    return cpdp;
}

int link_to_device_tcp(const std::string target_ip, uint32_t link_timeout)
{
#if (_WINDLL || _WINDOWS || _WIN32)
    // 网络初始化
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        printf("WSAStartup failed\n");
        return FS_EC_INIT_SOCK_FAILD;
    }
#endif

    // 初始化socket
    if (sock_ctrl == -1)
    {
        sock_ctrl = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    }

    struct sockaddr_in to_sockaddr;
    bool bSuc;

    to_sockaddr.sin_family = AF_INET;
    to_sockaddr.sin_addr.s_addr = inet_addr(target_ip.data());

    // 连接到控制服务
    to_sockaddr.sin_port = htons(CTRL_TARGET_PORT);
    bSuc = UnblockConnect(sock_ctrl, &to_sockaddr, link_timeout);
    if (!bSuc)
    {
#if (_WINDLL || _WINDOWS || _WIN32)
        printf("connect to ctrl-server error %d\n", WSAGetLastError());
        closesocket(sock_ctrl);
#else
        printf("connect to ctrl-server error %d\n", errno);
        close(sock_ctrl);
#endif
        sock_ctrl = -1;
        return FS_EC_LINK_CTRLSRV_FAILD;
    }

    g_link_net = true;
    // 获取最大包长
    g_max_pkg_len = get_max_pkg_size();

    return FS_EC_OK;
}

void break_link_tcp()
{
    g_link_net = false;
#if (_WINDLL || _WINDOWS || _WIN32)
    closesocket(sock_ctrl);
#else
    shutdown(sock_ctrl, SHUT_RDWR);
    close(sock_ctrl);
#endif
    sock_ctrl = -1;

#if (_WINDLL || _WINDOWS || _WIN32)
    WSACleanup(); // 清理 Winsock 资源
#endif

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

uint32_t get_max_pkg_size()
{
    std::string outData;
    int errCode = send_cmd_tcp(FS_HAL_CMD_GET_MAX_PKG_SIZE, (uint8_t *)"", 0, outData);

    if (errCode != FS_EC_OK)
    {
        return 0;
    }
    else
    {
        int32_t pkgSize = 0;
        memcpy(&pkgSize, outData.data(), 4);

        // 如果不是最大包长，尝试设置成最大
        if (pkgSize < MAX_TRANS_PKG_LEN)
        {
            if (set_max_pkg_size(MAX_TRANS_PKG_LEN))
            {
                pkgSize = MAX_TRANS_PKG_LEN;
            }
        }

        return (0 > pkgSize ? 0 : pkgSize);
    }
}

bool set_max_pkg_size(uint32_t maxPkgSize)
{
    uint8_t arrData[4] = {0};
    memcpy(arrData, &maxPkgSize, 4);
    std::string outData;

    int errCode = send_cmd_tcp(FS_HAL_CMD_SET_MAX_PKG_SIZE, arrData, 4, outData);
    if (errCode != FS_EC_OK)
    {
        return false;
    }
    else
    {
        return true;
    }
}

int send_cmd_tcp(uint16_t cmd_type, uint8_t *in_data, uint16_t data_len, std::string &out_data, uint32_t ms_timeout, uint16_t resend_cnt)
{
    if (!g_link_net)
    {
        return FS_EC_LINK_ERROR;
    }

    CtrlPrtDataPkg cpdp = pack_ctrl_prt_data_pkg(cmd_type, data_len, in_data);
    int pkgLen = PRT_HEAD_LEN + CTRL_CMD_HEAD_LEN + 2 + data_len;
    int errCode = FS_EC_SEND_FAILD;
    char *sendData = new char[pkgLen]{0};
    memcpy(sendData, &cpdp, PRT_HEAD_LEN + CTRL_CMD_HEAD_LEN + 2);
    memcpy(sendData + pkgLen - data_len, cpdp.data.data(), data_len);

    while (resend_cnt--)
    {
        int ret = ::send(sock_ctrl, sendData, pkgLen, 0);
        if (ret != pkgLen)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        char *recvBuf = new char[g_max_pkg_len]{0};
        int recvLen = ::recv(sock_ctrl, recvBuf, g_max_pkg_len, 0);
        if (recvLen > (int)(PRT_HEAD_LEN + CTRL_CMD_HEAD_LEN))
        {
            CtrlPrtDataPkg recvCpdp;
            memcpy(&recvCpdp, recvBuf, PRT_HEAD_LEN + CTRL_CMD_HEAD_LEN + 2);
            // 校准数据的有效性
            if (recvCpdp.ctrl_cmd_head.cmd == cpdp.ctrl_cmd_head.cmd && recvCpdp.head_crc == fs_crc_8_l((uint8_t *)&recvCpdp, PRT_HEAD_LEN + CTRL_CMD_HEAD_LEN) && recvLen == PRT_HEAD_LEN + CTRL_CMD_HEAD_LEN + 2 + recvCpdp.ctrl_cmd_head.len)
            {
                recvCpdp.data.assign(recvBuf + PRT_HEAD_LEN + CTRL_CMD_HEAD_LEN + 2, recvCpdp.ctrl_cmd_head.len);
                memcpy(&errCode, recvCpdp.data.data(), 4);
                if (recvCpdp.ctrl_cmd_head.len - 4 > 0)
                    out_data.assign(recvCpdp.data.data() + 4, recvCpdp.ctrl_cmd_head.len - 4);
                break;
            }
        }
    }

    delete[] sendData;
    return errCode;
}

bool UnblockConnect(int sock, struct sockaddr_in *to_addr, uint32_t link_timeout)
{
    // 设置为非阻塞方式连接
#if (_WINDLL || _WINDOWS || _WIN32)
    unsigned long ul = 1;
    int ret = ioctlsocket(sock, FIONBIO, (unsigned long *)&ul);
    if (ret == SOCKET_ERROR)
    {
        return false;
    }
#else
    int flags = fcntl(sock, F_GETFL,0);
    int ret = fcntl(sock, F_SETFL, flags |= O_NONBLOCK);
    if (ret == -1)
    {
        printf("set O_NONBLOCK failed!,errno=%d\n", errno);
        return false;
    }
#endif
    int linkRet = ::connect(sock, (sockaddr *)to_addr, sizeof(sockaddr));

    timeval tv;
    fd_set readfds, writefds, exceptfds;
    tv.tv_sec = link_timeout / 1000; // 秒
    tv.tv_usec = (link_timeout - tv.tv_sec * 1000) * 1000;

    FD_ZERO(&readfds);
    FD_ZERO(&writefds);
    FD_ZERO(&exceptfds);
    FD_SET(sock, &readfds);
    FD_SET(sock, &writefds);
    FD_SET(sock, &exceptfds);

    ret = select(sock+1, &readfds, &writefds, &exceptfds, &tv);
    if (ret <= 0)
    {
        return false;
    }

    // 设回阻塞模式
 #if (_WINDLL || _WINDOWS || _WIN32)   
    ul = 0;
    ret = ioctlsocket(sock, FIONBIO, (unsigned long *)&ul); // 返回之后，记得要设置为阻塞模式哟
#else
    flags = fcntl(sock, F_GETFL,0);
    flags &= ~O_NONBLOCK;
    ret = fcntl(sock, F_SETFL, flags);
    if (ret == -1)
    {
        printf("%s failed!,errno=%d\n",__FUNCTION__,errno);
        return false;
    }
#endif

    return true;
}
