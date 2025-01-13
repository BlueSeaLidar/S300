#include "FsTcpTransControl.h"
#include<string.h>
FsTcpTransControl* FsTcpTransControl::mInstance = nullptr;
std::mutex FsTcpTransControl::mtx;
FsTcpTransControl* FsTcpTransControl::getInstance()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (mInstance == nullptr) {
        mInstance = new FsTcpTransControl();
    }
    return mInstance;
}

FsTcpTransControl::FsTcpTransControl()
{
    
}

FsTcpTransControl::~FsTcpTransControl()
{

}

bool FsTcpTransControl::set_ipport(fs_ipport_t ipport_t)
{
    std::string outData;
    int errCode = send_cmd_tcp(FS_LIDAR_CMD_SET_DEV_IP_PORT, (uint8_t*)&ipport_t, sizeof(ipport_t), outData);
    if (errCode != FS_EC_OK)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool FsTcpTransControl::get_ipport(fs_ipport_t* ipport_t)
{
    std::string outData;
    int errCode = send_cmd_tcp(FS_LIDAR_CMD_GET_DEV_IP_PORT, (uint8_t*)"", 0, outData);
    if (errCode != FS_EC_OK)
    {
        return false;
    }
    else
    {
        if (outData.size() != sizeof(fs_ipport_t))
        {
            return false;
        }
        memcpy(ipport_t, outData.data(), sizeof(fs_ipport_t));
        return true;
    }
}