#ifndef NOMINALMPC_H
#define NOMINALMPC_H

#include <ConvexMPC/MPCLocoMotion.h>
#include <ConvexMPC/MPCSolver.h>

class NominalMPC : public MPCLocomotion
{
public:
  NominalMPC(double _dt, int horizon, Quadruped *quad);
  void run(ControlFSMData &data);

protected:
  MPCSolver solver;

  bool firstSwing[4];
  double swingTimeRemaining[4];
  double stand_traj[6];
  double foothold_offset[8];

  Vec3<double> world_position_desired;
  Vec3<double> rpy_int;
  Vec3<double> rpy_comp;
  Vec3<double> pFoot[4];
  double trajAll[12 * 36];

  Mat43<double> W; // W = [1, px, py]
  Vec3<double> a;  // a = [a_0, a_1, a_2]; z(x,y) = a_0 + a_1x + a_2y
  Vec4<double> pz;
  double ground_pitch;

  Vec3<double> v_des_robot;
  Vec3<double> v_des_world;
  Vec3<double> v_robot;
  double footSwingHeight = 0.1;
  Vec3<double> Pf;
  double alpha;
  Eigen::VectorXd Q;
  Vec3<double> KpJoint;
  Mat3<double> Kp, Kd, Kp_stance, Kd_stance;

  Vec3<double> f_ff[4];
  Vec4<double> swingTimes;
  FootSwingTrajectory<double> footSwingTrajectories[4];
  Vec4<double> contact_state;

  int iterationsBetweenMPC;
  double dtMPC;
  int iterationCounter = 0;
};

#endif