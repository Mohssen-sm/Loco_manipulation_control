#ifndef HIGHLEVELCMD_H
#define HIGHLEVELCMD_H

#include <iostream>
#include "../common/cppTypes.h"
#include "../common/enumClass.h"

struct HighlevelCmd
{
    float velocity[3];
    float omega[3];
    Vec3<double> manipulation_force;

    HighlevelCmd()
    {

        for (int i = 0; i < 3; i++)
        {
            velocity[i] = 0;
            omega[i] = 0;
            manipulation_force(i) = 0;
        }
    }
};

#endif