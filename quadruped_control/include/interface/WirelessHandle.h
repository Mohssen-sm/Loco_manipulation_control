#ifndef WIRELESSHANDLE_H
#define WIRELESSHANDLE_H

#include <sdk/include/unitree_legged_sdk/joystick.h>
#include "CmdPanel.h"
#include <sdk/include/unitree_legged_sdk/comm.h>

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