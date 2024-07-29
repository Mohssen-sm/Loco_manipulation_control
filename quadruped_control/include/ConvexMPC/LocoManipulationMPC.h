#ifndef LocoManipulationMPC_H
#define LocoManipulationMPC_H

#include <ConvexMPC/NominalMPC.h>
#include <ConvexMPC/MPCSolver.h>

class LocoManipulationMPC : public NominalMPC
{
public:
  LocoManipulationMPC(double _dt, int horizon, Quadruped *quad);
  void run(ControlFSMData &data);
};

#endif