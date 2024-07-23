#ifndef LocoManipulationMPC_H
#define LocoManipulationMPC_H

#include "MPCLocoMotion.h"
#include "MPCSolver.h"

class LocoManipulationMPC : public MPCLocomotion
{
public:
  LocoManipulationMPC(double _dt, int horizon, Quadruped *quad);
  void run(ControlFSMData &data);

  MPCSolver solver;
};

#endif