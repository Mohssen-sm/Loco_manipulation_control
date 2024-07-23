#include <ConvexMPC/MPCLocoMotion.h>

MPCLocomotion::MPCLocomotion(double _dt, int horizon, Quadruped *quad) : horizonLength(horizon),
                                                                         dt(_dt),
                                                                         galloping(horizonLength, Vec4<int>(0, 2, 7, 9), Vec4<int>(5, 5, 5, 5), "Galloping"),
                                                                         pronking(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(4, 4, 4, 4), "Pronking"),
                                                                         trotting(horizonLength, Vec4<int>(0, 5, 5, 0), Vec4<int>(6, 6, 6, 6), "trotting"),
                                                                         standing(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(10, 10, 10, 10), "Standing"),
                                                                         bounding(horizonLength, Vec4<int>(5, 5, 0, 0), Vec4<int>(5, 5, 5, 5), "Bounding"),
                                                                         walking(horizonLength, Vec4<int>(0, 3, 5, 8), Vec4<int>(5, 5, 5, 5), "Walking"),
                                                                         pacing(horizonLength, Vec4<int>(5, 0, 5, 0), Vec4<int>(5, 5, 5, 5), "Pacing"),
                                                                         two_foot_trot(horizonLength, Vec4<int>(0, 0, 5, 0), Vec4<int>(10, 10, 5, 5), "two_foot_trot"),
                                                                         static_walking(11, Vec4<int>(0, 2, 5, 8), Vec4<int>(7, 7, 7, 7), "static_walking")
{
  iterationsBetweenMPC = quad->gait_iteration;
  dtMPC = dt * iterationsBetweenMPC;

  rpy_int[2] = 0;
  for (int i = 0; i < 4; i++)
  {
    firstSwing[i] = true;
  }
  alpha = 4e-5;

  Q.resize(12);
  Q.setZero();
  KpJoint << 40, 40, 40;
  MPCUpdateLoop = 30;
}
