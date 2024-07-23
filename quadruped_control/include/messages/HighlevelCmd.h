#ifndef HIGHLEVELCMD_H
#define HIGHLEVELCMD_H

#include <iostream>
#include <common/cppTypes.h>
#include <common/enumClass.h>

struct HighlevelCmd
{
    Vec3<double> manipulation_force;
    Vec3<double> velocity_cmd;
    Vec3<double> omega_cmd;

    HighlevelCmd()
    {

        for (int i = 0; i < 3; i++)
        {
            velocity_cmd(i) = 0;
            omega_cmd(i) = 0;
            manipulation_force(i) = 0;
        }
    }
};

#endif