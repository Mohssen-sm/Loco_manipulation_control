#ifndef _RobotState
#define _RobotState

#include <eigen3/Eigen/Dense>
#include "common_types.h"

using Eigen::Quaternionf;

#include "common_types.h"
#include <common/Quadruped.h>
class RobotState
{
public:
    RobotState(Quadruped *quad);
    void set(flt *p, flt *v, flt *q, flt *w, flt *r, flt yaw);
    Eigen::Matrix<fpt, 3, 1> p, v, w;
    Eigen::Matrix<fpt, 3, 4> r_feet;
    Eigen::Matrix<fpt, 3, 3> R;
    Eigen::Matrix<fpt, 3, 3> R_yaw;
    Eigen::Matrix<fpt, 3, 3> I_body;
    Eigen::Matrix<fpt, 3, 3> I_world;
    Quaternionf q;
    fpt yaw;
    fpt mass;
};
#endif
