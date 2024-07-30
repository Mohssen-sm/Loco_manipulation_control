#ifndef IOSDK_H
#define IOSDK_H

#include <interface/IOInterface.h>
#include <sdk/include/unitree_legged_sdk/unitree_legged_sdk.h>

using namespace UNITREE_LEGGED_SDK;

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
    IOSDK(LeggedType robot, int cmd_panel_id);
    ~IOSDK() {
        loop_loco_manipulation.shutdown();
    }
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

    UDP _udp;
    Safety _control;
    LowCmd _lowCmd = {0};
    LowState _lowState = {0};
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
    LoopFunc loop_loco_manipulation;
};

#endif // IOSDK_H