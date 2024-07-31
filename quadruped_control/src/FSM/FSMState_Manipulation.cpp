#include <FSM/FSMState_Manipulation.h>

FSMState_Manipulation::FSMState_Manipulation(ControlFSMData *data)
    : FSMState(data, FSMStateName::MANIPULATION, "manipulation"),
      Cmpc(0.001, 10, data->_quadruped) {}

template <typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1)
{
    return (value - minLim) * (max - min) / (maxLim - minLim) + min;
}

void FSMState_Manipulation::enter()
{
    pitch = 0;
    roll = 0;
    counter = 0;
    Cmpc.firstRun = true;
    _data->_stateEstimator->run();
}

void FSMState_Manipulation::run()
{
    _data->_legController->updateData(_data->_lowState);
    _data->_stateEstimator->run();
    _userValue = _data->_lowState->userValue;
    // adjusthte velocity range based on the gait cycle time
    // v_des_body[0] = (double)invNormalize(_userValue.ly, -3.0, 3.0); // change this velocity limit for different robots
    v_des_body[0] = _userValue.vx; // Aliengo
    v_des_body[1] = _userValue.vy;
    turn_rate = _userValue.turn_rate;

    _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des_body, turn_rate);

    Cmpc.climb = true;
    Cmpc.setGaitNum(2);
    Cmpc.run(*_data);

    _data->_legController->updateCommand(_data->_lowCmd);
}

void FSMState_Manipulation::exit()
{
    counter = 0;
    _data->_interface->cmdPanel->setCmdNone();
}

FSMStateName FSMState_Manipulation::checkTransition()
{
    if (_lowState->userCmd == UserCommand::L1_X || manipulation2QP)
    {
        if (Cmpc.phase > 0.96)
        {
            std::cout << "transition from Manipulation to QP stand" << std::endl;
            manipulation2QP = false;
            return FSMStateName::QPSTAND;
        }
        else
        {
            manipulation2QP = true;
            return FSMStateName::MANIPULATION;
        }
    }
    else if (_lowState->userCmd == UserCommand::L2_B)
    {
        std::cout << "transition from Manipulation to passive" << std::endl;
        return FSMStateName::PASSIVE;
    }
    else
    {
        return FSMStateName::MANIPULATION;
    }
}
