/*!
 * @file Aliengo.h
 * @brief stores dynamics information
 * Leg 0: Left front; Leg 1: right front;
 * Leg 2: Left rear ; Leg 3: right rear;
 */ 
#ifndef PROJECT_QUADRUPED_H
#define PROJECT_QUADRUPED_H

#include <vector>
#include "cppTypes.h"
#include <string>
class Quadruped{
  public:
    void setQuadruped(int robot_id){
        robot_index = robot_id;
        if(robot_id == 1){ // Aliengo
            mass = 19;

            leg_offset_x = 0.2399;
            leg_offset_y = 0.051;
            leg_offset_z = 0.0;

            hipLinkLength = 0.0838;
            thighLinkLength = 0.25;
            calfLinkLength = 0.25;
        }
        if(robot_id == 2){ // A1
            mass = 12;

            leg_offset_x = 0.1805;
            leg_offset_y = 0.047;
            leg_offset_z = 0.0;

            hipLinkLength = 0.0838; // hip offset in const.xacro
            thighLinkLength = 0.2;
            calfLinkLength = 0.2;
        }
        if(robot_id == 3){ // B1
            mass = 55;

            leg_offset_x = 0.3455;
            leg_offset_y = 0.072;
            leg_offset_z = 0;

            hipLinkLength = 0.12675;
            thighLinkLength = 0.35;
            calfLinkLength = 0.35;
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

    Vec3<double> getHipLocation(int leg){
        assert(leg >=0 && leg <4);
        Vec3<double> pHip = Vec3<double>::Zero();
        if (leg == 0){
            pHip(0) = leg_offset_x;
            pHip(1) = -leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 1){
            pHip(0) = leg_offset_x;
            pHip(1) = leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 2){
            pHip(0) = -leg_offset_x;
            pHip(1) = -leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 3){
            pHip(0) = -leg_offset_x;
            pHip(1) = leg_offset_y;
            pHip(2) = leg_offset_z;
        }

        return pHip;
    };

};

#endif