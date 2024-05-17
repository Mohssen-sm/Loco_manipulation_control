#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>

#include "../include/common/ControlFSMData.h"
#include "../include/common/ContactEstimator.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/interface/IOROS.h"
#include "../include/interface/KeyBoard.h"
#include "../include/FSM/FSM.h"

bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
}

int main(int argc, char ** argv)
{
    IOInterface *ioInter;
    ros::init(argc, argv, "quad_control");
    
    std::string robot_name;
    
    ros::param::get("/robot_name", robot_name);
    std::cout << "robot name " << robot_name << std::endl;
    ioInter = new IOROS(robot_name);
    ros::Rate rate(1000);

    double dt = 0.001;
    Quadruped quad;
    quad.setQuadruped(3);
    LegController* legController = new LegController(quad);
    LowlevelCmd* cmd = new LowlevelCmd();
    LowlevelState* state = new LowlevelState();
    StateEstimate stateEstimate;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(state,
                                                                          legController->data,
                                                                          &stateEstimate);
                                                                        
    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<VectorNavOrientationEstimator>();
    stateEstimator->addEstimator<TunedKFPositionVelocityEstimator>();
    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = ioInter;
    _controlData->_lowCmd = cmd;
    _controlData->_lowState = state;

    FSM* _FSMController = new FSM(_controlData);

    signal(SIGINT, ShutDown);

    while(running)
    {
        _FSMController->run();
        rate.sleep();
    }

    delete _controlData;
    return 0;

}
