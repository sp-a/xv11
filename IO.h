#pragma once

#ifndef _IO_H_
#define _IO_H

#include <vector>
#include <list>
#include "customTypes.h"

int readOdometryFromFile(char*, odometry_t *, int);
int readLidarFromFile(char*, lidar_t*, int);

void clearFile(const char *);
int writeOdometryToFile(vector<odometry_t>, const char *);
void writeLidarToFile(vector<lidar_t>);

void dumpDataPointsToFile(point_t *, int, const char *);
void dumpSegmentsToFile(segment_t *segments, int num_features, const char*, int);
void dumpStateToFile(robot_state_t *robot, const char *filename, int id);

void dump_ocupancy_map(uint8_t *grid, int grid_size, const char *filename);

int openSerialPort(const char *devname);
int readFromSerialPort(int fd);
#endif