/*
 * math_utils.c
 *
 *  Created on: Mar 30, 2026
 *      Author: miyab
 */

#include "math_utils.h"

float deg_to_rad(float deg)
{
    return deg * DEG_TO_RAD_CONST;
}

float rad_to_deg(float rad)
{
    return rad * RAD_TO_DEG_CONST;
}
