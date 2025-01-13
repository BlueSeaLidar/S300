#pragma once
#include <string>
#include <mutex>
#include "TcpTrans.h"
#pragma pack(push, 1)

enum
{
    PS_PRT_VERSION = 0x0000,
    PS_PRT_POWER = 0x0010,
    PS_PRT_RX_WRITE = 0x0020,
    PS_PRT_RX_READ = 0x0021,
    PS_PRT_RXREG_WRITE = 0x0022,
    PS_PRT_RXREG_READ = 0x0023,
    PS_PRT_TX_CONTROL = 0x0030,
};
typedef struct fs_ipport {
    uint32_t lidar_ip;
    uint32_t host_ip;
    uint16_t lidar_port;
    uint16_t host_port;
    uint16_t reserved[16];
} fs_ipport_t;
#pragma pack(pop)

class FsTcpTransControl
{
public:
    static FsTcpTransControl* getInstance();
    ~FsTcpTransControl();
    bool set_ipport(fs_ipport_t ipport_t);
    bool get_ipport(fs_ipport_t* ipport_t);
private:
    FsTcpTransControl();
    static FsTcpTransControl* mInstance;
    static std::mutex mtx;
};
