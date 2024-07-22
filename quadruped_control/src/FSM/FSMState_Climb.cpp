#include "../../include/FSM/FSMState_Climb.h"

FSMState_Climb::FSMState_Climb(ControlFSMData *data)
               :FSMState(data, FSMStateName::CLIMB, "climb"),
               Cmpc(0.001, 10, data->_quadruped){}

template<typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}

void FSMState_Climb::enter()
{
    // v_des_body << 0, 0, 0;
    pitch = 0;
    roll = 0;
    // _data->_interface->zeroCmdPanel();
    counter = 0;
    _data->_desiredStateCommand->firstRun = true;
    Cmpc.firstRun = true;
    _data->_stateEstimator->run(); 
    // _data->_legController->zeroCommand();
}

void FSMState_Climb::run()
{
    _data->_legController->updateData(_data->_lowState);
    _data->_stateEstimator->run(); 
    _userValue = _data->_lowState->userValue;
    // adjusthte velocity range based on the gait cycle time
    v_des_body[0] = (double)invNormalize(_userValue.ly, -0.5, 0.5);
    v_des_body[1] = (double)invNormalize(_userValue.rx, 0.5, -0.5);
    turn_rate = (double)invNormalize(_userValue.lx, 2.0, -2.0);

    _data->_desiredStateCommand->setStateCommands(pitch, roll, v_des_body, turn_rate);
    Cmpc.climb = true;
    Cmpc.setGaitNum(2); 
    Cmpc.run(*_data);

    _data->_legController->updateCommand(_data->_lowCmd);  
}

void FSMState_Climb::exit()
{   
    // Cmpc.firstRun = true;
    counter = 0; 
    // _data->_legController->zeroCommand();
    // _data->_interface->zeroCmdPanel();
    _data->_legController->updateCommand(_data->_lowCmd);
    _data->_interface->cmdPanel->setCmdNone();
}

FSMStateName FSMState_Climb::checkTransition()
{
    if(_lowState->userCmd == UserCommand::START || climb2walking){
        if(Cmpc.phase > 0.95){
            std::cout << " transition from Climb to walk" << std::endl;
            climb2walking = false;
            return FSMStateName::WALKING;
        }
        else{
            climb2walking = true;
            return FSMStateName::CLIMB;
        }  
    }

    else if(_lowState->userCmd == UserCommand::L1_X){
        return FSMStateName::QPSTAND;
    }

    else if(_lowState->userCmd == UserCommand::L2_B){
        std::cout << "transition from PD stand to passive" << std::endl;
        return FSMStateName::PASSIVE;
    }
    
    else{
        return FSMStateName::CLIMB;
    }
}