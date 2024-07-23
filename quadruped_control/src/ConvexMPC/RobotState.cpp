#include <ConvexMPC/RobotState.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

RobotState::RobotState(Quadruped *quad)
{
    p = Eigen::Matrix<fpt, 3, 1>::Zero();
    v = Eigen::Matrix<fpt, 3, 1>::Zero();
    w = Eigen::Matrix<fpt, 3, 1>::Zero();
    r_feet = Eigen::Matrix<fpt, 3, 4>::Zero();
    R = Eigen::Matrix<fpt, 3, 3>::Identity();
    R_yaw = Eigen::Matrix<fpt, 3, 3>::Identity();
    q = Eigen::Quaternion<fpt>::Identity();
    yaw = 0;
    I_world = Eigen::Matrix<fpt, 3, 3>::Zero();

    I_body = quad->Ig.cast<fpt>().asDiagonal();
    mass = quad->mass;
}

void RobotState::set(flt *p_, flt *v_, flt *q_, flt *w_, flt *r_, flt yaw_)
{
    for (u8 i = 0; i < 3; i++)
    {
        this->p(i) = p_[i];
        this->v(i) = v_[i];
        this->w(i) = w_[i];
    }
    this->q.w() = q_[0];
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];
    this->yaw = yaw_;

    for (u8 rs = 0; rs < 3; rs++)
        for (u8 c = 0; c < 4; c++)
            this->r_feet(rs, c) = r_[rs * 4 + c];

    R = this->q.toRotationMatrix();

    R_yaw << cos(yaw_), -sin(yaw_), 0,
        sin(yaw_), cos(yaw_), 0,
        0, 0, 1;

    I_world = R_yaw * I_body * R_yaw.transpose();
}
