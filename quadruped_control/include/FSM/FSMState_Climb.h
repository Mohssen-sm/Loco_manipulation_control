#ifndef CLIMB_H
#define CLIMB_H

#include <FSM/FSMState.h>
#include <ConvexMPC/StairMPC.h>

class FSMState_Climb: public FSMState
{
    public:
        FSMState_Climb(ControlFSMData *data);
        ~FSMState_Climb(){}
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition();
    
    private:
        StairMPC Cmpc;
        int counter;
        Vec3<double> v_des_body;
        double turn_rate = 0;
        double pitch, roll;
        bool climb2walking = false;
};

#endif