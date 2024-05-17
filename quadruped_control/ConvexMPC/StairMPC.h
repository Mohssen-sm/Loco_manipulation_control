#ifndef STAIRMPC_H
#define STIARMPC_H

#include "MPCLocoMotion.h"
#include "MPCSolver.h"

class StairMPC : public MPCLocomotion {
public:
  StairMPC(double _dt, int _iterations_between_mpc, int horizon);

  void run(ControlFSMData& data);

  void updateMPCIfNeeded(int* mpcTable, ControlFSMData& data);

  MPCSolver solver;

};

#endif 