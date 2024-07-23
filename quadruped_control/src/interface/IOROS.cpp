#include <interface/IOROS.h>
#include <interface/KeyBoard.h>
#include <iostream>
#include <unistd.h>
#include <csignal>

inline void RosShutDown(int sig)
{
    ROS_INFO("ROS interface shutting down!");
    ros::shutdown();
}

IOROS::IOROS(Quadruped quad) : IOInterface(), _quad(quad)
{
    // start subscriber
    initRecv();
    ros::AsyncSpinner subSpinner(1); // one threads
    subSpinner.start();
    usleep(3000); // wait for subscribers start
    // initialize publisher
    initSend();

    signal(SIGINT, RosShutDown);

    cmdPanel = new KeyBoard();
    _nm.param<std::string>("tf_prefix", tf_prefix, "");
}

IOROS::~IOROS()
{
    ros::shutdown();
}

void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    sendCmd(cmd);
    recvState(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();

    if (planner_running)
    {
        for (int i = 0; i < 2; i++)
        {
            state->userValue.manipulation_force[i] = Highcmd.manipulation_force(i);
        }
        state->userValue.vx = Highcmd.velocity_cmd[0];
        state->userValue.vy = Highcmd.velocity_cmd[1];
        state->userValue.turn_rate = Highcmd.omega_cmd[2];
    }
    else
    {
        for (int i = 0; i < 2; i++)
        {
            state->userValue.manipulation_force[i] = 0;
        }
        state->userValue.vx = 0;
        state->userValue.vy = 0;
        state->userValue.turn_rate = 0;

        ROS_INFO_THROTTLE(1, "Waiting for new Target");
    }
}

void IOROS::sendCmd(const LowlevelCmd *lowCmd)
{
    for (int i = 0; i < 12; i++)
    {
        _lowCmd.motorCmd[i].mode = 0X0A; // alwasy set it to 0X0A
        _lowCmd.motorCmd[i].q = lowCmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq = lowCmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].tau = lowCmd->motorCmd[i].tau;
        _lowCmd.motorCmd[i].Kd = lowCmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].Kp = lowCmd->motorCmd[i].Kp;
    }
    // std::cout <<  _lowCmd.motorCmd[0].Kd << " " << _lowCmd.motorCmd[0].q << std::endl;
    for (int m = 0; m < 12; m++)
    {
        _servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
}

void IOROS::recvState(LowlevelState *state)
{
    for (int i = 0; i < 12; i++)
    {
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
    }
    for (int i = 0; i < 3; i++)
    {
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];

    for (int i = 0; i < 4; i++)
    {
        state->FootForce[i] = _lowState.footForce[i];
    }

    state->position[0] = _lowState.position.x;
    state->position[1] = _lowState.position.y;
    state->position[2] = _lowState.position.z;

    state->vWorld[0] = _lowState.velocity.x;
    state->vWorld[1] = _lowState.velocity.y;
    state->vWorld[2] = _lowState.velocity.z;

    _currentTime = ros::Time::now();

    _trunkTF.header.stamp = _currentTime;
    _trunkTF.header.frame_id = "world"; 
    _trunkTF.child_frame_id = tf_prefix + "/" + "base";

    _trunkTF.transform.translation.x = _lowState.position.x;
    _trunkTF.transform.translation.y = _lowState.position.y;
    _trunkTF.transform.translation.z = _lowState.position.z;

    _trunkTF.transform.rotation.w = state->imu.quaternion[0];
    _trunkTF.transform.rotation.x = state->imu.quaternion[1];
    _trunkTF.transform.rotation.y = state->imu.quaternion[2];
    _trunkTF.transform.rotation.z = state->imu.quaternion[3];
    _trunkTF_broadcaster.sendTransform(_trunkTF);

    _poseMsg.header.stamp = _currentTime;
    _poseMsg.header.frame_id = "base";

    _poseMsg.pose.pose.position.x = _lowState.position.x;
    _poseMsg.pose.pose.position.y = _lowState.position.y;
    _poseMsg.pose.pose.position.z = _lowState.position.z;
    _poseMsg.pose.covariance = _odom_pose_covariance;

    _poseMsg.pose.pose.orientation.w = state->imu.quaternion[0];
    _poseMsg.pose.pose.orientation.x = state->imu.quaternion[1];
    _poseMsg.pose.pose.orientation.y = state->imu.quaternion[2];
    _poseMsg.pose.pose.orientation.z = state->imu.quaternion[3];

    _pose_pub.publish(_poseMsg);
}

void IOROS::initSend()
{
    _servo_pub[0] = _nm.advertise<unitree_legged_msgs::MotorCmd>("FR_hip_controller/command", 1);
    _servo_pub[1] = _nm.advertise<unitree_legged_msgs::MotorCmd>("FR_thigh_controller/command", 1);
    _servo_pub[2] = _nm.advertise<unitree_legged_msgs::MotorCmd>("FR_calf_controller/command", 1);
    _servo_pub[3] = _nm.advertise<unitree_legged_msgs::MotorCmd>("FL_hip_controller/command", 1);
    _servo_pub[4] = _nm.advertise<unitree_legged_msgs::MotorCmd>("FL_thigh_controller/command", 1);
    _servo_pub[5] = _nm.advertise<unitree_legged_msgs::MotorCmd>("FL_calf_controller/command", 1);
    _servo_pub[6] = _nm.advertise<unitree_legged_msgs::MotorCmd>("RR_hip_controller/command", 1);
    _servo_pub[7] = _nm.advertise<unitree_legged_msgs::MotorCmd>("RR_thigh_controller/command", 1);
    _servo_pub[8] = _nm.advertise<unitree_legged_msgs::MotorCmd>("RR_calf_controller/command", 1);
    _servo_pub[9] = _nm.advertise<unitree_legged_msgs::MotorCmd>("RL_hip_controller/command", 1);
    _servo_pub[10] = _nm.advertise<unitree_legged_msgs::MotorCmd>("RL_thigh_controller/command", 1);
    _servo_pub[11] = _nm.advertise<unitree_legged_msgs::MotorCmd>("RL_calf_controller/command", 1);
    _pose_pub = _nm.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 50);
}

void IOROS::initRecv()
{
    _imu_sub = _nm.subscribe("trunk_imu", 1, &IOROS::imuCallback, this);
    _state_sub = _nm.subscribe("/gazebo/model_states", 1, &IOROS::StateCallback, this);
    _servo_sub[0] = _nm.subscribe("FR_hip_controller/state", 1, &IOROS::FRhipCallback, this);
    _servo_sub[1] = _nm.subscribe("FR_thigh_controller/state", 1, &IOROS::FRthighCallback, this);
    _servo_sub[2] = _nm.subscribe("FR_calf_controller/state", 1, &IOROS::FRcalfCallback, this);
    _servo_sub[3] = _nm.subscribe("FL_hip_controller/state", 1, &IOROS::FLhipCallback, this);
    _servo_sub[4] = _nm.subscribe("FL_thigh_controller/state", 1, &IOROS::FLthighCallback, this);
    _servo_sub[5] = _nm.subscribe("FL_calf_controller/state", 1, &IOROS::FLcalfCallback, this);
    _servo_sub[6] = _nm.subscribe("RR_hip_controller/state", 1, &IOROS::RRhipCallback, this);
    _servo_sub[7] = _nm.subscribe("RR_thigh_controller/state", 1, &IOROS::RRthighCallback, this);
    _servo_sub[8] = _nm.subscribe("RR_calf_controller/state", 1, &IOROS::RRcalfCallback, this);
    _servo_sub[9] = _nm.subscribe("RL_hip_controller/state", 1, &IOROS::RLhipCallback, this);
    _servo_sub[10] = _nm.subscribe("RL_thigh_controller/state", 1, &IOROS::RLthighCallback, this);
    _servo_sub[11] = _nm.subscribe("RL_calf_controller/state", 1, &IOROS::RLcalfCallback, this);

    _foot_force_sub[0] = _nm.subscribe("visual/FR_foot_contact/the_force", 1, &IOROS::FRfootCallback, this);
    _foot_force_sub[1] = _nm.subscribe("visual/FL_foot_contact/the_force", 1, &IOROS::FLfootCallback, this);
    _foot_force_sub[2] = _nm.subscribe("visual/RR_foot_contact/the_force", 1, &IOROS::RRfootCallback, this);
    _foot_force_sub[3] = _nm.subscribe("visual/RL_foot_contact/the_force", 1, &IOROS::RLfootCallback, this);

    _manipulation_force_sub = _nm.subscribe("wrench", 1, &IOROS::ManiForceCallback, this);
    _object_sub[0] = _nm.subscribe("cmd_vel", 1, &IOROS::cmdvelCallback, this);
    _object_sub[1] = _nm.subscribe("contactPoint", 1, &IOROS::poseCallback, this);

    // create a ROS timer
    timer = _nm.createTimer(ros::Duration(0.5), &IOROS::timerCallback, this);
}

void IOROS::StateCallback(const gazebo_msgs::ModelStates &msg)
{
    int robot_index;
    for (int i = 0; i < msg.name.size(); i++)
    {
        if (msg.name[i] == _nm.getNamespace().substr(1))
        {
            robot_index = i;
        }
    }

    _lowState.position.x = msg.pose[robot_index].position.x;
    _lowState.position.y = msg.pose[robot_index].position.y;
    _lowState.position.z = msg.pose[robot_index].position.z;

    _lowState.velocity.x = msg.twist[robot_index].linear.x;
    _lowState.velocity.y = msg.twist[robot_index].linear.y;
    _lowState.velocity.z = msg.twist[robot_index].linear.z;
}

void IOROS::imuCallback(const sensor_msgs::Imu &msg)
{
    _lowState.imu.quaternion[0] = msg.orientation.w;
    _lowState.imu.quaternion[1] = msg.orientation.x;
    _lowState.imu.quaternion[2] = msg.orientation.y;
    _lowState.imu.quaternion[3] = msg.orientation.z;

    Eigen::Quaterniond quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    rotmat = quat.toRotationMatrix();

    _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg.angular_velocity.z;

    _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
}

void IOROS::FRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[0].mode = msg.mode;
    _lowState.motorState[0].q = msg.q;
    _lowState.motorState[0].dq = msg.dq;
    _lowState.motorState[0].tauEst = msg.tauEst;
}

void IOROS::FRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[1].mode = msg.mode;
    _lowState.motorState[1].q = msg.q;
    _lowState.motorState[1].dq = msg.dq;
    _lowState.motorState[1].tauEst = msg.tauEst;
}

void IOROS::FRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[2].mode = msg.mode;
    _lowState.motorState[2].q = msg.q;
    _lowState.motorState[2].dq = msg.dq;
    _lowState.motorState[2].tauEst = msg.tauEst;
}

void IOROS::FLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[3].mode = msg.mode;
    _lowState.motorState[3].q = msg.q;
    _lowState.motorState[3].dq = msg.dq;
    _lowState.motorState[3].tauEst = msg.tauEst;
}

void IOROS::FLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[4].mode = msg.mode;
    _lowState.motorState[4].q = msg.q;
    _lowState.motorState[4].dq = msg.dq;
    _lowState.motorState[4].tauEst = msg.tauEst;
}

void IOROS::FLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[5].mode = msg.mode;
    _lowState.motorState[5].q = msg.q;
    _lowState.motorState[5].dq = msg.dq;
    _lowState.motorState[5].tauEst = msg.tauEst;
}

void IOROS::RRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[6].mode = msg.mode;
    _lowState.motorState[6].q = msg.q;
    _lowState.motorState[6].dq = msg.dq;
    _lowState.motorState[6].tauEst = msg.tauEst;
}

void IOROS::RRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[7].mode = msg.mode;
    _lowState.motorState[7].q = msg.q;
    _lowState.motorState[7].dq = msg.dq;
    _lowState.motorState[7].tauEst = msg.tauEst;
}

void IOROS::RRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[8].mode = msg.mode;
    _lowState.motorState[8].q = msg.q;
    _lowState.motorState[8].dq = msg.dq;
    _lowState.motorState[8].tauEst = msg.tauEst;
}

void IOROS::RLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[9].mode = msg.mode;
    _lowState.motorState[9].q = msg.q;
    _lowState.motorState[9].dq = msg.dq;
    _lowState.motorState[9].tauEst = msg.tauEst;
}

void IOROS::RLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[10].mode = msg.mode;
    _lowState.motorState[10].q = msg.q;
    _lowState.motorState[10].dq = msg.dq;
    _lowState.motorState[10].tauEst = msg.tauEst;
}

void IOROS::RLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[11].mode = msg.mode;
    _lowState.motorState[11].q = msg.q;
    _lowState.motorState[11].dq = msg.dq;
    _lowState.motorState[11].tauEst = msg.tauEst;
}

void IOROS::FRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    _lowState.footForce[0] = msg.wrench.force.z;
}

void IOROS::FLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    _lowState.footForce[1] = msg.wrench.force.z;
}

void IOROS::RRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    _lowState.footForce[2] = msg.wrench.force.z;
}

void IOROS::RLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    _lowState.footForce[3] = msg.wrench.force.z;
}

void IOROS::ManiForceCallback(const geometry_msgs::Wrench &msg)
{
    Eigen::Vector3d force_body = (Eigen::Vector3d() << msg.force.x, msg.force.y, msg.force.z).finished();
    Highcmd.manipulation_force = rotmat * force_body;
    msg_received = true;
    // ROS_INFO("I heard: x =%f, y=%f, z=%f", Highcmd.manipulation_force[0], Highcmd.manipulation_force[1], Highcmd.manipulation_force[2]);
}

void IOROS::cmdvelCallback(const geometry_msgs::Twist &msg)
{
    cmd_body << msg.linear.x, msg.linear.y, 0.0;
    Highcmd.omega_cmd[2] = msg.angular.z;
    msg_received = true;
}

void IOROS::poseCallback(const geometry_msgs::Pose &msg)
{
    msg_received = true;

    Eigen::Vector3d pose_world = (Eigen::Vector3d() << _lowState.position.x, _lowState.position.y, 0).finished();
    Eigen::Vector3d contact_point_world = (Eigen::Vector3d() << msg.position.x, msg.position.y, 0).finished();
    Eigen::Vector3d distance_body = rotmat.transpose() * (contact_point_world - pose_world);

    Highcmd.velocity_cmd[0] = 2 * (distance_body[0] - _quad.leg_offset_x);  
    Highcmd.velocity_cmd[1] = cmd_body[1] + 3 * (distance_body[1]); // magic number
    //   ROS_INFO("I heard: x =%f, y=%f, z=%f", Highcmd.velocity_cmd[0], Highcmd.velocity_cmd[1], Highcmd.omega_cmd[2]);
}

void IOROS::timerCallback(const ros::TimerEvent &)
{
    if (msg_received)
    {
        msg_received = false;
        planner_running = true;
    }
    else
    {
        planner_running = false;
    }
}