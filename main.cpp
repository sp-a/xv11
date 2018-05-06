#include <iostream>
#include "stdio.h"
#include <assert.h>
#include <vector>
#include <list>
#include "customTypes.h"
#include "Eigen/Dense"
#include "IO.h"
#include "Odometry.h"
#include "Lidar.h"
#include "Detect.h"
#include "Geometry.h"
#include "Association.h"
#include "Grid.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace Eigen;
using namespace std;
using namespace cv;

////// Global variables

odometry_t odometry[MAX_ODOMETRY_SAMPLES];
lidar_t lidar[MAX_LIDAR_SAMPLES];
point_t data_points[MAX_LIDAR_SAMPLES];
point_t backup[MAX_LIDAR_SAMPLES];

segment_t segments[MAX_NEW_SEGMENTS];
point_t landmarks[MAX_NEW_SEGMENTS - 1];

segment_t sg_db[MAX_DB_FEATURES];
point_t   lm_db[MAX_DB_FEATURES];
int age_sg[MAX_DB_FEATURES];
int age_lm[MAX_DB_FEATURES];

int match_sg[MAX_NEW_SEGMENTS];
int match_lm[MAX_NEW_SEGMENTS - 1];

float inn_sg[MAX_NEW_SEGMENTS];
float inn_lm[MAX_NEW_SEGMENTS * 2];

int num_sg_db = 0;
int num_lm_db = 0;

uint8_t grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE];
uint8_t g_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE];
uint8_t l_grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE];
uint8_t pp_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE];

point_t l_points[LOCAL_GRID_SIZE*LOCAL_GRID_SIZE];


int main()
{
	FeatureDetector detect(10, 40, (float)0.03, 50);

	int num_odometry_samples = readOdometryFromFile(odometry, MAX_ODOMETRY_SAMPLES);
	int num_lidar_samples    = readLidarFromFile(lidar, MAX_LIDAR_SAMPLES);

	clearFile("data_points.txt");
	clearFile("landmarks.txt");
	clearFile("segments.txt");
	clearFile("state.txt");
	clearFile("grid.txt");

	init_trig_lut();

	robot_state_t robot;
	initializeOdometry(&robot, odometry, num_odometry_samples);


	for (int iter = 0; iter < num_lidar_samples; ++iter) {

		float time = lidar[iter].ts; // current time
		int dangle, dx, dy;

		evolve(&robot, time);

		// Feature extraction
		int num_points = convertLidarToSamplePoint(lidar[iter], data_points, num_lidar_samples);


		if (1)
		{
			get_local_ocupancy_grid(data_points, num_points, grid, 0.1, LOCAL_GRID_MAX_RANGE);
			//dump_ocupancy_map(&grid[0][0], LOCAL_GRID_MAX_RANGE, "grid.txt");
			float angle = robot.Q;
			int rx = roundf(robot.pos.x / 0.1);
			int ry = roundf(robot.pos.y / 0.1);
			extract_local_grid(g_grid, GLOBAL_GRID_MAX_RANGE, l_grid, LOCAL_GRID_MAX_RANGE, angle, rx, ry);
			//num_points = extract_local_points(g_grid, GLOBAL_GRID_MAX_RANGE, l_points, LOCAL_GRID_MAX_RANGE, angle, rx, ry);
			int cost, matches;
			scan_to_map(l_grid, LOCAL_GRID_MAX_RANGE, data_points, num_points, &dangle, &dx, &dy, &cost, &matches);
			if (matches > 150)
			{
				printf("update: %d %d %d %d\n", dangle, dx, dy, matches);
				robot.Q += dangle * DEGREES_TO_RADIAN;
				//robot.pos.x += dx * 0.05 * 0.2;
				//robot.pos.y += dy * 0.05 * 0.2;
			}
			printf("cost: %d %d %d %d %d\n", matches, cost, dx, dy, dangle);
			update_map(g_grid, GLOBAL_GRID_MAX_RANGE, grid, LOCAL_GRID_MAX_RANGE, robot.Q, roundf(robot.pos.x / 0.1), roundf(robot.pos.y / 0.1));

		}
		else
		{

			int num_segments = detect.extractSegmentsRansac(segments,
				data_points,
				num_points);
			num_segments = detect.filterSegments(segments, num_segments, PI / 3, 0.5);
			int num_landmarks = detect.intersectSegments(segments, num_segments, landmarks);

			printf("num landmarks: %d\n", num_landmarks);
			get_local_ocupancy_grid(data_points, num_points, grid, 0.05, LOCAL_GRID_MAX_RANGE);

			segment_t tem_sg[32];
			for (int i = 0; i < num_segments; ++i)
				tem_sg[i] = segments[i];
			transform_segments(segments, num_segments, robot.pos.x, robot.pos.y, robot.Q);
			match_segments(segments, num_segments, sg_db, num_sg_db, match_sg, inn_sg, robot);	

			float k_angle = 1.0;
			float sumq = 0.0;
			int count = 0;
			for (int i = 0; i < num_segments; ++i)
			{
				if (match_sg[i] == -1) continue;
				sumq += inn_sg[i];
				count++;
				//robot.Q += k_angle * inn_sg[i];
				/*if (robot.Q > 2 * PI) robot.Q -= 2 * PI;
				if (robot.Q < 0) robot.Q += 2 * PI;*/
			}
			if (count)
			{
				sumq /= count;
				robot.Q += k_angle * sumq;
			}

			transform_points(data_points, num_points, robot.pos.x, robot.pos.y, robot.Q);
			transform_segments(tem_sg, num_segments, robot.pos.x, robot.pos.y, robot.Q);
			dumpSegmentsToFile(segments, num_segments, "segments.txt", iter);
			dumpDataPointsToFile(data_points, num_points, "data_points.txt");

			transform_points(landmarks, num_landmarks, robot.pos.x, robot.pos.y, robot.Q);
			match_landmarks(landmarks, num_landmarks, lm_db, num_lm_db, match_lm, inn_lm, robot);
			dumpDataPointsToFile(landmarks, num_landmarks, "landmarks.txt");

			float k_tr = 0.0;
			float sumx = 0, sumy = 0;
			count = 0;
			for (int i = 0; i < num_landmarks; ++i)
			{
				sumx += inn_lm[i * 2];
				sumy += inn_lm[i * 2 + 1];
				count++;
			}
			if (count)
			{
				sumx /= count;
				sumy /= count;
				printf("dx: %f , dy %f\n", sumx, sumy);
				robot.pos.x += sumx * k_tr;
				robot.pos.y += sumy * k_tr;
			}

			// Remove old features
			/*count = 0;
			for (int i = 0; i < num_sg_db; ++i)
				if (++sg_db[i].age < 100)
					sg_db[count++] = sg_db[i];
			num_sg_db = count;

			count = 0;
			for (int i = 0; i < num_lm_db; ++i)
				if (++lm_db[i].age < 100)
					lm_db[count++] = lm_db[i];
			num_lm_db = count;*/

			// Add new features
			for (int i = 0; i < num_segments; ++i)
				if (match_sg[i] == -1)
				{
					segments[i].age = 0;
					sg_db[num_sg_db++] = segments[i];
				}
			for (int i = 0; i < num_landmarks; ++i)
				if (match_lm[i] == -1)
				{
					landmarks[i].age = 0;
					lm_db[num_lm_db++] = landmarks[i];
				}

			
			count = 0;
			for (int i = 0; i < num_segments; ++i)
				if (match_sg[i] == -1)
					tem_sg[count++] = segments[i];
			//dumpSegmentsToFile(tem_sg, count, "segments.txt", iter);
			update_map(g_grid, GLOBAL_GRID_MAX_RANGE, grid, LOCAL_GRID_MAX_RANGE, robot.Q, (int)(robot.pos.x / 0.05), (int)(robot.pos.y / 0.05));
		}
		
	}

	//post_process_grid_erode(g_grid, pp_grid, GLOBAL_GRID_SIZE);

	dump_ocupancy_map(&g_grid[0][0], GLOBAL_GRID_MAX_RANGE, "grid.txt");

	cout << "Done!" << endl;
	int wait;
	cin >> wait;

	return 0;
}