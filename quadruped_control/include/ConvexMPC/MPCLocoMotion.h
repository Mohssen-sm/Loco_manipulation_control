#ifndef MPCLOCOMOTION_H
#define MPCLOCOMOTION_H

#include <fstream>
#include <stdio.h>
#include <iostream>
#include <common/Utilities/Timer.h>
#include <common/Math/orientation_tools.h>
#include <common/cppTypes.h>
#include <common/FootSwingTrajectory.h>
#include <common/ControlFSMData.h>
#include <ConvexMPC/Gait.h>

using namespace std;

using Eigen::Array4d;
using Eigen::Array4f;
using Eigen::Array4i;

class MPCLocomotion
{
public:
  MPCLocomotion(double _dt, int horizon, Quadruped *quad) : horizonLength(horizon),
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
  }

  virtual void run(ControlFSMData &data) = 0;

  void setGaitNum(int gaitNum)
  {
    gaitNumber = gaitNum;
    return;
  }

  double phase;
  double t = 0.0;
  bool firstRun = true;
  bool climb = 0;

protected:
  int gaitNumber;
  int horizonLength;
  double dt;
  int MPCUpdateLoop;

  Gait trotting, bounding, pacing, walking, galloping, pronking, standing, two_foot_trot, static_walking;
};

#endif