//
// Created by Ken_n on 2022/2/13.
//

#ifndef STANDARDROBOTBASICCODE_MY_AHRS_H
#define STANDARDROBOTBASICCODE_MY_AHRS_H

#include "Fusion.h"

FusionVector3 gyroscopeSensitivity = {
        .axis.x = 0.06103515625f,
        .axis.y = 0.06103515625f,
        .axis.z = 0.06103515625f,
}; // replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet

FusionVector3 accelerometerSensitivity = {
        .axis.x = 1.0f,
        .axis.y = 1.0f,
        .axis.z = 1.0f,
}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet
#endif //STANDARDROBOTBASICCODE_MY_AHRS_H
