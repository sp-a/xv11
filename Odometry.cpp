
#include "Odometry.h"
#include "Geometry.h"
#include "iostream"

using namespace std;

static int o_index;
static int num_samples;
static odometry_t *odometry_data;


void initializeOdometry(robot_state_t *robot, odometry_t *odometry, int n_samples)
{
	memset(robot, 0, sizeof(robot));

	o_index = 0;
	odometry_data = odometry;
	num_samples = n_samples;
}

void getCurrentOdometry(robot_state_t *robot, float time, float *odom)
{
	odom[0] = 0;
	odom[1] = 0;
	odom[2] = 0;

	if (o_index == 0)
	{
		o_index++;
	}
	else
	{
		while (o_index < num_samples && odometry_data[o_index].ts < time)
		{

			float v = odometry_data[o_index].tv;
			float w = odometry_data[o_index].rv;
			float dt = odometry_data[o_index].ts - robot->ts;
			float Q = robot->Q;

			if (w)
			{
				odom[0] += -v / w*sin(Q) + v / w*sin(Q + w*dt);
				odom[1] += v / w*cos(Q) - v / w*cos(Q + w*dt);
				odom[2] += w*dt;
			}
			else
			{
				odom[0] += v*dt * cos(Q);
				odom[1] += v*dt * sin(Q);
				odom[2] += 0;
			}

			// printf("%d: v %f w %f dt %f\n",o_index,  v, w, dt);
			// printf("odom: %f %f %f\n" , odom[0], odom[1], odom[2]);
			// printf("robot.Q: %f\n" , robot->Q);

			robot->ts = odometry_data[o_index].ts;
			o_index ++;
		}
	}
}
// #define CORRECTED_LOG 1
void evolve(robot_state_t *robot, float time)
{
	float time_diff;

	if (o_index == 0)
	{
		if (0)
		{
			robot->pos.x = odometry_data[0].x;
			robot->pos.y = odometry_data[0].y;
			robot->Q = odometry_data[0].Q;
		}
		else
		{
			robot->pos.x = 0;
			robot->pos.y = 0;
			robot->Q = 0;
		}

		o_index++;
		return;
	}

	while (o_index < num_samples && odometry_data[o_index].ts < time)
	{
		if (0)
		{
			float dx = odometry_data[o_index].x - odometry_data[o_index - 1].x;
			float dy = odometry_data[o_index].y - odometry_data[o_index - 1].y;
			float dQ = odometry_data[o_index].Q - odometry_data[o_index - 1].Q;
			float Q = odometry_data[o_index - 1].Q;

			float drot1 = atan2(dy, dx) - Q;
			float dtrans = sqrt(dx*dx + dy*dy);
			float drot2 = dQ - drot1;

			robot->pos.x += dtrans * cos(robot->Q + drot1);
			robot->pos.y += dtrans * sin(robot->Q + drot1);
			robot->Q += dQ;

#ifdef CORRECTED_LOG
			robot->pos.x = odometry_data[o_index].x;
			robot->pos.y = odometry_data[o_index].y;
			robot->Q = odometry_data[o_index].Q;
#endif
			robot->ts = odometry_data[o_index].ts;
		}
		else
		{
			predict(robot, odometry_data[o_index]);
		}

		o_index++;
	}
}

void predict(robot_state_t* robot, odometry_t odom)
{
	float v = odom.tv;
	float w = odom.rv;
	float dt = odom.ts - robot->ts;
	float Q = robot->Q;

	if (w)
	{
		robot->pos.x += -v / w*sin(Q) + v / w*sin(Q + w*dt);
		robot->pos.y += v / w*cos(Q) - v / w*cos(Q + w*dt);
		robot->Q += w*dt;
	}
	else
	{
		robot->pos.x += v*dt * cos(Q);
		robot->pos.y += v*dt * sin(Q);
	}

	robot->ts = odom.ts;
}

