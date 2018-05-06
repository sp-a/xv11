#pragma once

#include "customTypes.h"

#define PI 3.14159265359


float distance(point_t a, point_t b);
void transform_point(point_t *p, float dx, float fy, float Q);
void transform_points(point_t *p, int num_points, float dx, float fy, float Q);
void transform_segments(segment_t *segments, int num_segments, float x, float y, float angle);
