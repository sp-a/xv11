#pragma once

#include "customTypes.h"

void initializeOdometry(robot_state_t *robot, odometry_t *odometry, int num_samples);

void evolve(robot_state_t *robot, float time);

void predict(robot_state_t* robot, odometry_t odom);