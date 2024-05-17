#ifndef Manipulation_H
#define Manipulation_H

#include "FSMState.h"
#include "../../ConvexMPC/LocoManipulationMPC.h"

class FSMState_Manipulation: public FSMState
{
    public:
        FSMState_Manipulation(ControlFSMData *data);
        ~FSMState_Manipulation(){}
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition();
    
    private:
        LocoManipulationMPC Cmpc;
        int counter;
        Vec3<double> v_des_body;
        double turn_rate = 0;
        double pitch, roll;
        bool manipulation2QP = false;
};

#endif