#pragma once
#include "customTypes.h"

void match_segments(segment_t *new_sg, int num_new_sg, segment_t *old_sg,
	int num_old_sg, int *match, float *inn, robot_state_t robot);
void match_landmarks(point_t *new_p, int num_new_p, point_t *old_p,
	int num_old_p, int *match, float *inn, robot_state_t robot);
