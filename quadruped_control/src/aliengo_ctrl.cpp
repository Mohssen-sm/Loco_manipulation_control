#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

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

int main(int argc, char **argv)
{
    IOInterface *ioInter;
    ros::init(argc, argv, "aliengo_ctrl");
    ioInter = new IOROS("aliengo");
    ros::Rate rate(1000);

    double dt = 0.001;
    Quadruped quad;
    quad.setQuadruped(1);
    LegController *legController = new LegController(quad);
    LowlevelCmd *lowCmd = new LowlevelCmd();
    LowlevelState *lowState = new LowlevelState();
    StateEstimate stateEstimate;
    StateEstimatorContainer *stateEstimator = new StateEstimatorContainer(lowState,
                                                                          legController->data,
                                                                          &stateEstimate);

    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<VectorNavOrientationEstimator>();
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();
    DesiredStateCommand *desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData *_controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = ioInter;
    _controlData->_lowCmd = lowCmd;
    _controlData->_lowState = lowState;

    FSM *_FSMController = new FSM(_controlData);

    signal(SIGINT, ShutDown);

    while (running)
    {
        _FSMController->run();
        rate.sleep();
    }

    delete _controlData;
    return 0;
}
