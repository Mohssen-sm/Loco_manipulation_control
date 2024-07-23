#ifndef NOMINALMPC_H
#define NOMINALMPC_H

#include <ConvexMPC/MPCLocoMotion.h>
#include <ConvexMPC/MPCSolver.h>

class NominalMPC : public MPCLocomotion
{
public:
  NominalMPC(double _dt, int horizon, Quadruped *quad);
  void run(ControlFSMData &data);
  void updateMPCIfNeeded(int *mpcTable, ControlFSMData &data);

  MPCSolver solver;
};

#endif