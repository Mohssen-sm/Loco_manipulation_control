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

class WirelessHandle : public CmdPanel{
public:
    WirelessHandle();
    ~WirelessHandle(){}
    void receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState);
private:
    xRockerBtnDataStruct _keyData;
    // LPFilter *_L2Value, *_lxValue, *_lyValue, *_rxValue, *_ryValue;
};

#endif  // WIRELESSHANDLE_H