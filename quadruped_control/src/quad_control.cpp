#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>

#include <common/ControlFSMData.h>
#include <common/ContactEstimator.h>
#include <common/OrientationEstimator.h>
#include <common/PositionVelocityEstimator.h>
#include <interface/IOROS.h>
#include <interface/KeyBoard.h>
#include <FSM/FSM.h>

bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
}

int main(int argc, char **argv)
{
    std::vector<std::string> programArgs{};
    ::ros::removeROSArgs(argc, argv, programArgs);
    if (programArgs.size() <= 1)
    {
        throw std::runtime_error("No robot type specified. Aborting.");
    }
    std::string robot_name(programArgs[1]);

    if (robot_name != "aliengo" && robot_name != "a1" && robot_name != "go1")
    {
        throw std::runtime_error("Invalid robot type specified. Aborting.");
    }

    bool cheater = true;
    if (programArgs.size() > 2)
    {
        cheater = std::stoi(programArgs[2]);
    }

    ros::init(argc, argv, robot_name + "_control");

    IOInterface *ioInter;
    ioInter = new IOROS();
    ros::Rate rate(1000);

    double dt = 0.001;
    Quadruped quad(robot_name);

    LegController *legController = new LegController(quad);
    LowlevelCmd *cmd = new LowlevelCmd();
    LowlevelState *state = new LowlevelState();
    StateEstimate stateEstimate;
    StateEstimatorContainer *stateEstimator = new StateEstimatorContainer(state,
                                                                          legController->data,
                                                                          &stateEstimate);

    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<VectorNavOrientationEstimator>();
    if (cheater)
    {
        stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();
    }
    else
    {
        stateEstimator->addEstimator<TunedKFPositionVelocityEstimator>();
    }
    DesiredStateCommand *desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData *_controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = ioInter;
    _controlData->_lowCmd = cmd;
    _controlData->_lowState = state;

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
