#include "Association.h"
#include "Geometry.h"
#include "math.h"
#include <stdlib.h> 

void match_segments(segment_t *new_sg, int num_new_sg, segment_t *old_sg,
	int num_old_sg, int *match, float *inn, robot_state_t robot)
{
	for (int i = 0; i < num_new_sg; ++i)
	{
		// compute thresholds
		// 0.2m for features 1m away from the robot
		float dist_th = distance(robot.pos, new_sg[i].middle) * 0.5;

		float min_dist = 1000000.0;
		match[i] = -1;
		for (int j = 0; j < num_old_sg; ++j)
		{		
			float dist = distance(new_sg[i].middle, old_sg[j].middle);
			if (dist < dist_th && abs(new_sg[i].slope - old_sg[j].slope) < 0.5)
			{
				if (dist < min_dist)
				{
					min_dist = dist;
					match[i] = j;
					inn[i] = old_sg[j].slope - new_sg[i].slope;
				}
			}
		}
	}
}

void match_landmarks(point_t *new_p, int num_new_p, point_t *old_p,
	int num_old_p, int *match, float *inn, robot_state_t robot)
{
	int pos;

	for (int i = 0; i < num_new_p; ++i)
	{
		// compute thresholds
		// 0.2m for features 1m away from the robot
		float dist_th = distance(robot.pos, new_p[i]) * 0.1;

		float min_dist = 1000000.0;
		match[i] = -1;
		for (int j = 0; j < num_old_p; ++j)
		{
			float dist = distance(new_p[i], old_p[j]);
			if (dist < min_dist)
			{
				min_dist = dist;
				pos = j;
			}
		}

		if (min_dist < dist_th)
		{
			match[i] = pos;
			inn[2 * i] = old_p[pos].x - new_p[i].x;
			inn[2 * i + 1] = old_p[pos].y - new_p[i].y;
		}
	}
}