#ifndef WHEEL_DRIVER_BLD300B_CONTROL_WHEEL_H
#define WHEEL_DRIVER_BLD300B_CONTROL_WHEEL_H

#include "main.h"
#include "dac.h"
#include "math.h"
#include "odometry.h"
#include <stdlib.h>

#define MIN_SPEED           80.0f
#define CONTROL_LOOP_SKIP   100

void stop();
void emergency_stop();
void enable_wheels();
void set_target_speed(uint8_t wheel, float speed);
void force_wheels();
void control_iterate(uint8_t wheel, ODOMETRY_MEASUREMENT odom);

#endif //WHEEL_DRIVER_BLD300B_CONTROL_WHEEL_H
