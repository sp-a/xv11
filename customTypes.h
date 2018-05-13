
#ifndef _CUSTOM_TYPES_H_
#define _CUSTOM_TYPES_H_

// #include "Eigen/Dense"
#include <stdint.h>
#include <list>
#include "stdlib.h"
#include "math.h"
#include "string.h"
#include <stdio.h>

// using namespace Eigen;
using namespace std;

#define DEGREES_TO_RADIAN 0.0174532925
#define RADIAN_TO_DEGREE  57.2957795

///// Global defines
#define MAX_LIDAR_DATA_PEE_SAMPLE       400

#define FEAT_RANGE_DETECT_THRESH       10.0 // 10 m
#define FEAT_SPIKE_DETECT_THRESH        0.5

#define FEAT_DIST_MATCH_THRESH          0.2 // 20cm
#define FEAT_SEEN_MATCH_THRESH           1  // feature has been seen at least 10 times

#define FEAT_AGE_FORGET_THRESH		     40  // forget feature after 40 samples
#define MAX_FEATURES					  40
#define ROBOT_POS_SIZE					  3
#define MAX_STATE_SIZE	(ROBOT_POS_SIZE + 2 * MAX_FEATURES)

#define MAX_ODOMETRY_SAMPLES 4096
#define MAX_LIDAR_SAMPLES    4096
#define MAX_DB_FEATURES  	 (8*1024)
#define MAX_NEW_SEGMENTS     64

#define LOCAL_GRID_MAX_RANGE 250
#define PADDING_SIZE 10
#define LOCAL_GRID_SIZE (LOCAL_GRID_MAX_RANGE * 2 + 1)
#define LOCAL_GRID_PADDED_SIZE (LOCAL_GRID_SIZE + 2 * PADDING_SIZE)
#define GLOBAL_GRID_MAX_RANGE 1000
#define GLOBAL_GRID_SIZE (GLOBAL_GRID_MAX_RANGE * 2 + 1)


typedef struct {
	float x, y;      // absolute position
	float Q;         // heading
	float ts;        // seconds?
	float tv;
	float rv;
}odometry_t;

typedef struct {
	int laser_type;
	float start_angle;
	float field_of_view;
	float angular_resolution;
	float maximum_range;
	float accuracy;
	int remission_mode;
	int num_readings;
	float ts;
	float data[MAX_LIDAR_DATA_PEE_SAMPLE];
	bool has_odometry;
	odometry_t odom;
}lidar_t;

typedef struct point {
	float x, y;
	int age;
}point_t;

// typedef struct {
// 	float range, bearing;

// 	float getRange()
// 	{
// 		return range;
// 	}

// 	float getLocalX()
// 	{
// 		return range * cos(bearing);
// 	}

// 	float getLocalY()
// 	{
// 		return range * sin(bearing);
// 	}

// }observation_t;

typedef struct {
	point_t middle;
	point_t edges[2];
	float slope;

	float m, n; //  y = m * x + n
	float var;

	int age;
}segment_t;

typedef struct {
	float x, y, Q;
}position_t;

typedef struct {
	float vx, vy, vQ;
}velocity_t;

typedef float angle_t;

typedef struct{
	point_t pos;
	angle_t Q;
	velocity_t vel;
	float ts;
}robot_state_t;

#endif