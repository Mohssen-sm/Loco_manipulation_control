#ifndef PROJECT_QUADRUPED_H
#define PROJECT_QUADRUPED_H

#include <vector>
#include "cppTypes.h"
#include <string>
class Quadruped
{
public:
    Quadruped(std::string robot_name)
    {
        if (robot_name == "aliengo")
        {
            robot_index = 1;
        }
        else if (robot_name == "a1")
        {
            robot_index = 2;
        }
        else if (robot_name == "go1")
        {
            robot_index = 3;
        }
        else
        {
            throw std::runtime_error("Invalid robot type specified. Aborting.");
        }
        setQuadruped();
    }
    void setQuadruped()
    {
        if (robot_index == 1)
        { // Aliengo
            mass = 19;
            Ig << 0.050874, 0.64036, 0.65655;
            COM_height = 0.4;

            leg_offset_x = 0.2399;
            leg_offset_y = 0.051;
            leg_offset_z = 0.0;

            hipLinkLength = 0.0838;
            thighLinkLength = 0.25;
            calfLinkLength = 0.25;

            gait_iteration = 50;
        }
        else if (robot_index == 2)
        { // A1
            mass = 12;
            Ig << 0.0168, 0.0565, 0.064;
            COM_height = 0.3;

            leg_offset_x = 0.1805;
            leg_offset_y = 0.047;
            leg_offset_z = 0.0;

            hipLinkLength = 0.0838; // hip offset in const.xacro
            thighLinkLength = 0.2;
            calfLinkLength = 0.2;

            gait_iteration = 30;
        }
        else if (robot_index == 3)
        { // Go1
            mass = 12.84;
            Ig << 0.0792, 0.2085, 0.2265;
            COM_height = 0.3;

            leg_offset_x = 0.1881;
            leg_offset_y = 0.04675;
            leg_offset_z = 0.0;

            hipLinkLength = 0.08; // hip offset in const.xacro
            thighLinkLength = 0.213;
            calfLinkLength = 0.213;

            gait_iteration = 30;
        }
    }
    int robot_index; // 1 for Aliengo, 2 for A1
    double hipLinkLength;
    double thighLinkLength;
    double calfLinkLength;
    double leg_offset_x;
    double leg_offset_y;
    double leg_offset_z;
    double mass;
    Vec3<double> Ig;
    int gait_iteration;
    double COM_height;

    Vec3<double> getHipLocation(int leg)
    {
        assert(leg >= 0 && leg < 4);
        Vec3<double> pHip = Vec3<double>::Zero();
        if (leg == 0)
        {
            pHip(0) = leg_offset_x;
            pHip(1) = -leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 1)
        {
            pHip(0) = leg_offset_x;
            pHip(1) = leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 2)
        {
            pHip(0) = -leg_offset_x;
            pHip(1) = -leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 3)
        {
            pHip(0) = -leg_offset_x;
            pHip(1) = leg_offset_y;
            pHip(2) = leg_offset_z;
        }

        return pHip;
    };
};

#endif