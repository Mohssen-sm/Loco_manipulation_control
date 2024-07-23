#include <iostream>
#include <ConvexMPC/LocoManipulationMPC.h>
#include <common/Math/orientation_tools.h>

using namespace ori;

/* =========================== Controller ============================= */
LocoManipulationMPC::LocoManipulationMPC(double _dt, int horizon, Quadruped *quad)
    : MPCLocomotion(_dt, horizon, quad),
      solver(_dt * quad->gait_iteration, horizon, 0.2, 350, quad)
{
  solver.setup_LocoManipulation(horizonLength, 15);

  if (quad->robot_index == 1) // aliengo
  {
    // Q << 8.0, 5.0, 10, 6, 1.5, 15, 0, 0, 0.3, 0.2, 0.2, 0.2;
    Q << 15.0, 12.0, 10, 1.5, 1.5, 60, 0, 0, 0.3, 0.2, 0.2, 0.2;
    footSwingHeight = 0.12;
    KpJoint << 80, 80, 80;
    MPCUpdateLoop = 30;
  }
  else if (quad->robot_index == 2) // a1
  {
    Q << 0.5, 0.5, 10, 2.5, 2.5, 20, 0, 0, 0.3, 0.4, 0.4, 0.4;
  }
  else if (quad->robot_index == 3) // go1
  {
    Q << 15.0, 12.0, 10, 1.5, 1.5, 35, 0, 0, 0.3, 0.2, 0.2, 0.2;
    footSwingHeight = 0.1;
    KpJoint << 40, 40, 40;
    MPCUpdateLoop = 30;
  }
}

void LocoManipulationMPC::run(ControlFSMData &data)
{
  auto seResult = data._stateEstimator->getResult();
  auto &stateCommand = data._desiredStateCommand;

  // pick gait
  Gait *gait = &trotting;
  if (gaitNumber == 1)
    gait = &bounding;
  else if (gaitNumber == 2)
    gait = &trotting;
  else if (gaitNumber == 3)
    gait = &walking;
  else if (gaitNumber == 4)
    gait = &pacing;
  else if (gaitNumber == 5)
    gait = &galloping;
  else if (gaitNumber == 6)
    gait = &pronking;
  else if (gaitNumber == 7)
    gait = &standing;
  else if (gaitNumber == 8)
    gait = &two_foot_trot;
  else if (gaitNumber == 9)
    gait = &static_walking;

  v_des_robot << stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], 0;
  v_des_world = seResult.rBody.transpose() * v_des_robot;

  v_robot = seResult.vBody;

  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = data._quadruped->COM_height;

  if (fabs(v_robot[0]) > .2) // avoid dividing by zero
  {
    rpy_int[1] += dt * (stateCommand->data.stateDes[4] - seResult.rpy[1]) / v_robot[0];
  }
  if (fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt * (stateCommand->data.stateDes[3] - seResult.rpy[0]) / v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0]; // turn off for pronking

  for (int i = 0; i < 4; i++)
  {
    pFoot[i] = seResult.position + seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + data._legController->data[i].p);
  }

  for (int i = 0; i < 4; i++)
  {
    W.row(i) << 1, footSwingTrajectories[i].getInitialPosition()[0], footSwingTrajectories[i].getInitialPosition()[1];
    pz[i] = footSwingTrajectories[i].getInitialPosition()[2];
  }
  a = W.transpose() * W * (W.transpose() * W * W.transpose() * W).inverse() * W.transpose() * pz;
  ground_pitch = acos(-1 / sqrt(a[1] * a[1] + a[2] * a[2] + 1)) - 3.14;
  // std::cout << "ground pitch: " << ground_pitch << std::endl;
  if (pz[0] < pz[2])
  {
    ground_pitch = -ground_pitch;
  }
  if (abs(pz[0] - pz[2]) < 0.01)
  {
    ground_pitch = 0;
  }

  // some first time initialization
  if (firstRun)
  {
    std::cout << "Run MPC" << std::endl;
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    v_des_robot << 0, 0, 0;
    v_des_world << 0, 0, 0;

    iterationCounter = 0;
    for (int i = 0; i < 4; i++)
    {
      footSwingTrajectories[i].setHeight(0.1);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }

    swingTimes[0] = dtMPC * gait->_swing;
    swingTimes[1] = dtMPC * gait->_swing;
    swingTimes[2] = dtMPC * gait->_swing;
    swingTimes[3] = dtMPC * gait->_swing;
    firstRun = false;
  }

  contact_state = gait->getContactSubPhase();
  double side_sign[4] = {-1, 1, -1, 1};
  for (int i = 0; i < 4; i++)
  {

    if (firstSwing[i])
    {
      swingTimeRemaining[i] = swingTimes[i];
    }
    else
    {
      swingTimeRemaining[i] -= dt;
    }
    footSwingTrajectories[i].setHeight(footSwingHeight);
    Vec3<double> offset(0.0, side_sign[i] * 0.06, 0);

    Vec3<double> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);
    Vec3<double> pYawCorrected = coordinateRotation(CoordinateAxis::Z, -stateCommand->data.stateDes[11] * gait->_stance * dtMPC / 2) * pRobotFrame;

    Pf = seResult.position +
         seResult.rBody.transpose() * pYawCorrected + seResult.vWorld * swingTimeRemaining[i];

    double p_rel_max = 0.2;
    double pfx_rel = seResult.vWorld[0] * .5 * gait->_stance * dtMPC +
                     .03 * (seResult.vWorld[0] - v_des_world[0]) +
                     (0.5 * seResult.position[2] / 9.81) * (seResult.vWorld[1] * stateCommand->data.stateDes[11]);
    double pfy_rel = seResult.vWorld[1] * .5 * gait->_stance * dtMPC +
                     .03 * (seResult.vWorld[1] - v_des_world[1]) +
                     (0.5 * seResult.position[2] / 9.81) * (-seResult.vWorld[0] * stateCommand->data.stateDes[11]);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel;
    Pf[2] = 0.0;

    footSwingTrajectories[i].setFinalPosition(Pf);
  }

  // calc gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  int *mpcTable = gait->mpc_gait();
  phase = gait->_phase;

  // gait
  Vec4<double> contactStates = gait->getContactSubPhase();
  Vec4<double> swingStates = gait->getSwingSubPhase();

  for (int foot = 0; foot < 4; foot++)
  {

    double contactState = contactStates(foot);
    double swingState = swingStates(foot);

    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
      Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();

      Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      data._legController->commands[foot].feedforwardForce << 0, 0, 0;
      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;

      data._legController->commands[foot].kpJoint.diagonal() = KpJoint;
      data._legController->commands[foot].kdJoint.diagonal() << 2, 2, 2;
      // account for early contact
      if (data._stateEstimator->getResult().contactEstimate(foot) != 0)
      {
        mpcTable[foot] = 1;
      }
    }
    else if (contactState > 0 || data._stateEstimator->getResult().contactEstimate(foot) != 0) // foot is in stance
    {
      firstSwing[foot] = true;
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
      Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
      Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      data._legController->commands[foot].kpJoint.diagonal() << 0, 0, 0;
      data._legController->commands[foot].kdJoint.diagonal() << 0.5, 0.5, 0.5;

      data._legController->commands[foot].feedforwardForce = f_ff[foot];

      if (gaitNumber == 8)
      {
        for (int i = 0; i < 2; i++)
          data._legController->commands[i].vDes << 0, 0, 0;
      }
    }
  }

  if ((iterationCounter % MPCUpdateLoop) == 0)
  {
    double r[12];
    for (int i = 0; i < 12; i++)
      r[i] = pFoot[i % 4][i / 4] - seResult.position[i / 4];

    double xStart = world_position_desired[0];
    double yStart = world_position_desired[1];

    if (xStart - seResult.position(0) > 0.1)
      xStart = seResult.position(0) + 0.1;
    if (seResult.position(0) - xStart > 0.1)
      xStart = seResult.position(0) - 0.1;

    if (yStart - seResult.position(1) > 0.1)
      yStart = seResult.position(1) + 0.1;
    if (seResult.position(1) - yStart > 0.1)
      yStart = seResult.position(1) - 0.1;

    world_position_desired[0] = xStart;
    world_position_desired[1] = yStart;

    double trajInitial[12] = {rpy_comp[0] + stateCommand->data.stateDes[3], rpy_comp[1] + stateCommand->data.stateDes[4], stateCommand->data.stateDes[5],
                              world_position_desired[0], world_position_desired[1], world_position_desired[2],
                              0, 0, stateCommand->data.stateDes[11],
                              v_des_world[0], v_des_world[1], v_des_world[2]};
    if (climb)
    {
      trajInitial[1] += ground_pitch;
    }

    for (int i = 0; i < horizonLength; i++)
    {
      for (int j = 0; j < 12; j++)
        trajAll[12 * i + j] = trajInitial[j];
      if (i != 0)
      {
        trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
        trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
        trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * stateCommand->data.stateDes[11];
      }
    }

    solver.set_manipulation_force(data._lowState->userValue.manipulation_force);

    solver.update_problem_data(seResult.position.data(), seResult.vWorld.data(), seResult.orientation.data(),
     seResult.omegaWorld.data(), r, seResult.rpy[2], Q.data(), trajAll, alpha, mpcTable);

    for (int leg = 0; leg < 4; leg++)
    {
      Vec3<double> f;
      for (int axis = 0; axis < 3; axis++)
      {
        f[axis] = solver.get_solution(leg * 3 + axis);
      }
      f_ff[leg] = -seResult.rBody * f;
    }
  }

  iterationCounter++;
}