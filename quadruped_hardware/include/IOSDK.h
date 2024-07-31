#ifndef IOSDK_H
#define IOSDK_H

#include <interface/IOInterface.h>
#include "WirelessHandle.h"
#include "interface/KeyBoard.h"

#ifdef GO1
#include "sdk_3_8_0/include/unitree_legged_sdk/quadruped.h"
#include "sdk_3_8_0/include/unitree_legged_sdk/udp.h"
#include "sdk_3_8_0/include/unitree_legged_sdk/safety.h"
#include "sdk_3_8_0/include/unitree_legged_sdk/loop.h"
#else
#include "sdk_3_3_1/include/unitree_legged_sdk/quadruped.h"
#include "sdk_3_3_1/include/unitree_legged_sdk/udp.h"
#include "sdk_3_3_1/include/unitree_legged_sdk/safety.h"
#include "sdk_3_3_1/include/unitree_legged_sdk/loop.h"
#endif


struct DataRecv
{
    float quaternion[4];
    float position[3];
    float velocity[3];
    float omega[3];
    float force[3];
};
struct DataSend
{
    bool contact;
};
class IOSDK : public IOInterface
{
public:
    IOSDK(UNITREE_LEGGED_SDK::LeggedType robot, int cmd_panel_id);
    ~IOSDK()
    {
        loop_loco_manipulation.shutdown();
    }
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

    UNITREE_LEGGED_SDK::UDP _udp;
    UNITREE_LEGGED_SDK::Safety _control;
    UNITREE_LEGGED_SDK::LowCmd _lowCmd = {0};
    UNITREE_LEGGED_SDK::LowState _lowState = {0};
    void socketSendRecv();

private:
    int setupSocket(int port);
    int serverSocket;
    int clientSocket;
    DataRecv _dataRecv;
    DataSend _dataSend;
    bool planner_running = false;
    HighlevelCmd Highcmd;
    Eigen::Matrix3d rotmat;
    UNITREE_LEGGED_SDK::LoopFunc loop_loco_manipulation;
};

#endif // IOSDK_H