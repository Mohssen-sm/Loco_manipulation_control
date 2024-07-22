#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include "../common/ControlFSMData.h"
#include "../common/cppTypes.h"
#include "../common/enumClass.h"
#include "../interface/CmdPanel.h"
#include "../messages/LowLevelCmd.h"
#include "../messages/LowlevelState.h"

class FSMState
{
public:
    FSMState(ControlFSMData *data, FSMStateName stateName, std::string stateNameStr) : _data(data), _stateName(stateName), _stateNameStr(stateNameStr)
    {
        _lowCmd = _data->_lowCmd;
        _lowState = _data->_lowState;
    }

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual FSMStateName checkTransition() { return FSMStateName::INVALID; }

    FSMStateName _stateName;
    std::string _stateNameStr;

protected:
    ControlFSMData *_data;
    FSMStateName _nextStateName;

    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;
    UserValue _userValue;
};

#endif // FSMSTATE_H