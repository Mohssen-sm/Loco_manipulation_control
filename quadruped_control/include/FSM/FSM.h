#ifndef FSM_H
#define FSM_H

#include "FSMState.h"
#include "FSMState_Passive.h"
#include "FSMState_PDStand.h"
#include "FSMState_QPStand.h"
#include "FSMState_Walking.h"
#include "FSMState_ThreeFoot.h"
#include "FSMState_Climb.h"
#include "FSMState_Manipulation.h"
#include "../common/enumClass.h"

struct FSMStateList{
    FSMState *invalid;
    FSMState_Passive *passive;
    FSMState_PDStand *pdstand;
    FSMState_QPStand *qpstand;
    FSMState_Walking *walking;
    FSMState_ThreeFoot *threefoot;
    FSMState_Climb * climb;
    FSMState_Manipulation *manipulation;
   
    void deletePtr(){
        delete invalid;
        delete passive;
        delete qpstand;
        delete walking;
        delete threefoot;
        delete climb;
        delete manipulation;
    }  
};

class FSM{
    public:
        FSM(ControlFSMData *data);
        ~FSM();
        void initialize();
        void run();
    private:
        FSMState* getNextState(FSMStateName stateName);
        bool checkSafty();
        ControlFSMData *_data;
        FSMState *_currentState;
        FSMState *_nextState;
        FSMStateName _nextStateName;
        FSMStateList _stateList;
        FSMMode _mode;
        long long _startTime;
        int count;
};

#endif