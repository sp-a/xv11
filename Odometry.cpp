
#include "Odometry.h"
#include "Geometry.h"

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
			robot->Q += dQ;
			while (robot->Q - 2 * PI > 0) robot->Q -= 2 * PI;
			while (robot->Q < 0) robot->Q += 2 * PI;

			robot->pos.x += dx; // *cos(robot->Q) - dy * sin(robot->Q);
			robot->pos.y += dy; // *sin(robot->Q) + dy * cos(robot->Q); */

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


// Convert from absolute values to deltas (SIMULATOR) - won't be needed in real world 
//void preprocessOdometry(odometry_t current, odometry_t &delta)
//{
//	static odometry_t prev = { 0, 0, 0 };
//	static bool first = 1;
//
//	if (first) {
//		prev = current;
//		first = 0;
//	}
//
//	delta.x = current.x - prev.x;
//	delta.y = current.y - prev.y;
//	delta.Q = current.Q - prev.Q;
//
//
//
//	prev = current;
//}
//
//void processOdometry(state_t &s, odometry_t delta)
//{
//	// Evolve
//	s.X(0, 0) += delta.x;
//	s.X(1, 0) += delta.y;
//	s.X(2, 0) += delta.Q;
//
//	/*if (gState.X(2) > 3.14)
//	gState.X(2) -= 2 * 3.14;
//	if (gState.X(2) < -3.14)
//	gState.X(2) += 2 * 3.14;*/
//
//	// Jacobian of the prediction model
//	s.A << 1, 0, -delta.y,
//		0, 1, delta.x,
//		0, 0, 1;
//
//	// SLAM jacobians for feature integration
//	s.Jxr << 1, 0, -delta.y,
//		0, 1, delta.x;
//
//	// This will be sensor dependent
//	odometry_t noise;
//	noise.x = delta.x * 0.1; // 10%
//	noise.y = delta.y * 0.1; // 10%
//	noise.Q = delta.Q * 0.1; // 10%
//							 // Process noise
//							 /*gState.Q << noise.x * noise.x, noise.x * noise.y, noise.x * noise.Q,
//							 noise.y * noise.x, noise.y * noise.y, noise.y * noise.Q,
//							 noise.Q * noise.x, noise.Q * noise.y, noise.Q * noise.Q;*/
//	s.Q.setZero();
//	s.Q(0, 0) = noise.x * noise.x;
//	s.Q(1, 1) = noise.y * noise.y;
//	s.Q(2, 2) = noise.Q * noise.Q;
//
//	//cout << "ODOM q:\n" << gState.Q << endl;
//
//	// Update robot covariance
//	MatrixXf cov = s.P.block(0, 0, 3, 3);
//	MatrixXf res = s.A * cov * s.A.transpose() + s.Q; // No Jacobian for Q ?
//													  //cout << "ODOM res:\n" << res << endl;
//
//	s.P.block(0, 0, 3, 3) = res;
//
//	// Update robot to feature cross corelation
//	for (int ft = 0; ft < s.fcount; ft++)
//	{
//		MatrixXf rtf = s.P.block<3, 2>(0, 3 + ft * 2);
//		MatrixXf res = s.A * rtf;
//
//		s.P.block<3, 2>(0, 3 + ft * 2) = res;
//		s.P.block<2, 3>(3 + ft * 2, 0) = res.transpose();
//	}
//}
