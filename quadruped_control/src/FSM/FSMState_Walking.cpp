#include <FSM/FSMState_Walking.h>

FSMState_Walking::FSMState_Walking(ControlFSMData *data)
    : FSMState(data, FSMStateName::WALKING, "walking"),
      Cmpc(0.001, 10, data->_quadruped) {}

template <typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1)
{
    return (value - minLim) * (max - min) / (maxLim - minLim) + min;
}

void FSMState_Walking::enter()
{
    // v_des_body << 0, 0, 0;
    pitch = 0;
    roll = 0;
    //  _data->_interface->zeroCmdPanel();
    counter = 0;
    Cmpc.firstRun = true;
    _data->_stateEstimator->run();
    // _data->_legController->zeroCommand();
}

void FSMState_Walking::run()
{
    _data->_legController->updateData(_data->_lowState);
    _data->_stateEstimator->run();
    _userValue = _data->_lowState->userValue;
    // adjusthte velocity range based on the gait cycle time
    // v_des_body[0] = (double)invNormalize(_userValue.ly, -3.0, 3.0); // change this velocity limit for different robots
    v_des_body[0] = (double)invNormalize(_userValue.ly, -1.0, 1.0); // Aliengo
    if (abs(v_des_body[0]) < 0.01)
    {
        v_des_body[0] = 0.15;
    }

    v_des_body[1] = (double)invNormalize(_userValue.rx, 0.5, -0.5);
    turn_rate = (double)invNormalize(_userValue.lx, 2.0, -2.0);

    _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des_body, turn_rate);

    if (counter > 2000 && counter < 6000)
    {
        _data->_quadruped->COM_height = 0.35 - ((counter - 2000.0) / 4000.0) * 0.13;
    }

    else if (counter > 6000 && counter < 10000)
    {
        _data->_quadruped->COM_height = 0.22;
    }

    else if (counter > 10000 && counter < 16000)
    {
        _data->_quadruped->COM_height = 0.22 + ((counter - 10000.0) / 4000.0) * 0.13;
    }

    else{
        _data->_quadruped->COM_height = 0.35;
    }

    Cmpc.climb = true;
    Cmpc.setGaitNum(2);
    Cmpc.run(*_data);

    _data->_legController->updateCommand(_data->_lowCmd);
    counter++;
}

void FSMState_Walking::exit()
{
    counter = 0;
    _data->_interface->cmdPanel->setCmdNone();
}

FSMStateName FSMState_Walking::checkTransition()
{
    if (_lowState->userCmd == UserCommand::L1_X || walking2QP)
    {
        if (Cmpc.phase > 0.96)
        {
            std::cout << "transition from walk to QP stand" << std::endl;
            walking2QP = false;
            return FSMStateName::QPSTAND;
        }
        else
        {

            walking2QP = true;
            return FSMStateName::WALKING;
        }
    }
    else if (_lowState->userCmd == UserCommand::L2_B)
    {
        std::cout << "transition from walking to passive" << std::endl;
        return FSMStateName::PASSIVE;
    }
    else
    {
        return FSMStateName::WALKING;
    }
}