#pragma once

#include "customTypes.h"

#define PI 3.14159265359

float sg_length(segment_t seg);
float distance(point_t a, point_t b);
float slope_dif(float slope0, float slope1);
void transform_point(point_t *p, float dx, float fy, float Q);
void transform_points(point_t *p, int num_points, float dx, float fy, float Q);

void transform_segment(segment_t *sg, float x, float y, float angle);
void transform_segments(segment_t *segments, int num_segments, float x, float y, float angle);
