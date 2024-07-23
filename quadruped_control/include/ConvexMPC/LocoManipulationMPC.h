#ifndef LocoManipulationMPC_H
#define LocoManipulationMPC_H

#include <ConvexMPC/MPCLocoMotion.h>
#include <ConvexMPC/MPCSolver.h>

class LocoManipulationMPC : public MPCLocomotion
{
public:
  LocoManipulationMPC(double _dt, int horizon, Quadruped *quad);
  void run(ControlFSMData &data);

  MPCSolver solver;
};

#endif