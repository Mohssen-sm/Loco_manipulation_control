#ifndef LocoManipulationMPC_H
#define LocoManipulationMPC_H

#include "MPCLocoMotion.h"
#include "MPCSolver.h"

class LocoManipulationMPC : public MPCLocomotion
{
public:
  LocoManipulationMPC(double _dt, int _iterations_between_mpc, int horizon);
  void run(ControlFSMData &data);
  void updateMPCIfNeeded(int* mpcTable, ControlFSMData& data);

  MPCSolver solver;
  float manipulation_force[3] = {0, 0, 0};
};

#endif