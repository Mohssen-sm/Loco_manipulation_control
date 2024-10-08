#ifndef PDSTAND_H
#define PDSTAND_H

#include "FSMState.h"

class FSMState_PDStand: public FSMState
{
    public:
        FSMState_PDStand(ControlFSMData *data);
        ~FSMState_PDStand(){}
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition();

    private:
        double _targetPos[12] =  {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 
                                  0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

        double _startPos[12];
        double _duration = 1000;
        double _percent = 0;
};

#endif