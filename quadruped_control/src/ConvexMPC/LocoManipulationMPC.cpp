#include <iostream>
#include <ConvexMPC/LocoManipulationMPC.h>
#include <common/Math/orientation_tools.h>

using namespace ori;

/* =========================== Controller ============================= */
LocoManipulationMPC::LocoManipulationMPC(double _dt, int horizon, Quadruped *quad)
    : NominalMPC(_dt, horizon, quad)
{
  solver.setup_LocoManipulation(horizonLength, 15);
}

void LocoManipulationMPC::run(ControlFSMData &data)
{
    solver.set_manipulation_force(data._lowState->userValue.manipulation_force);
    NominalMPC::run(data);
}