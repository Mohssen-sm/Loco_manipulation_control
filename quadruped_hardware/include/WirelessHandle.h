#ifndef WIRELESSHANDLE_H
#define WIRELESSHANDLE_H

#ifdef GO1
#include "sdk_3_8_0/include/unitree_legged_sdk/comm.h"
#include "sdk_3_8_0/include/unitree_legged_sdk/joystick.h"
#else
#include "sdk_3_3_1/include/unitree_legged_sdk/comm.h"
#include "sdk_3_3_1/include/unitree_legged_sdk/unitree_joystick.h"
#endif

#include <boost/bind.hpp>
#include <interface/CmdPanel.h>
#include <termios.h>

class WirelessHandle : public CmdPanel
{
public:
    WirelessHandle(UNITREE_LEGGED_SDK::LowState *lowState);
    ~WirelessHandle();

private:
    static void *runWirelessHandle(void *arg);
    void *run(void *arg);
    xRockerBtnDataStruct _keyData;
    UNITREE_LEGGED_SDK::LowState *_lowState;
    pthread_t _tid;
    struct termios _oldSettings, _newSettings;
};

#endif // WIRELESSHANDLE_H