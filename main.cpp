#include <iostream>
#include "stdio.h"
#include <assert.h>
#include <vector>
#include <list>
#include "customTypes.h"
// #include "Eigen/Dense"
#include "IO.h"
#include "Odometry.h"
#include "Lidar.h"
#include "Detect.h"
#include "Geometry.h"
#include "Association.h"
#include "Grid.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "icpPointToPlane.h"
#include "icpPointToPoint.h"

// using namespace Eigen;
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
uint8_t l_grid[LOCAL_GRID_PADDED_SIZE][LOCAL_GRID_PADDED_SIZE];
uint8_t pp_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE];

point_t l_points[LOCAL_GRID_SIZE*LOCAL_GRID_SIZE];
double icp_template[MAX_LIDAR_SAMPLES * 2];
double icp_model[LOCAL_GRID_PADDED_SIZE * LOCAL_GRID_PADDED_SIZE];
int tp_num, mp_num;


double angle_diff(double a, double b)
{
	double q1 = min(a,b);
	double q2 = max(a,b);

	return min(abs(q1-q2), abs(q1 + 2 * PI -q2));
}


int main()
{
	FeatureDetector detect(10, 30, (float)0.01, 50);

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

	for(int i = 0; i < GLOBAL_GRID_SIZE ; ++i )
		for(int j = 0; j < GLOBAL_GRID_SIZE; ++j)
			g_grid[i][j] = 128; // unknown



	for (int iter = 0; iter < num_lidar_samples; ++iter) {

		float time = lidar[iter].ts; // current time
		int dangle, dx, dy;

		// evolve(&robot, time);
		// while(robot.Q > 2* PI)
		//  	robot.Q -= 2 * PI;
		// while(robot.Q <  0)
		//  	robot.Q += 2 * PI;

		float odom[3];
		getCurrentOdometry(&robot, time, odom);

		int num_points = convertLidarToSamplePoint(lidar[iter], data_points, num_lidar_samples);

		if (1)
		{
			int index = iter % 2;
			get_local_ocupancy_grid(data_points, num_points, grid, 0.05, LOCAL_GRID_MAX_RANGE);

			for(int i = 0 ; i < num_points ; ++i )
			{
				data_points[i].x = roundf(data_points[i].x / 0.05);
				data_points[i].y = roundf(data_points[i].y / 0.05);
			}

			float angle = robot.Q;
			int rx = (int)roundf(robot.pos.x / 0.05);
			int ry = (int)roundf(robot.pos.y / 0.05);
			extract_local_grid(g_grid, GLOBAL_GRID_MAX_RANGE, l_grid, LOCAL_GRID_MAX_RANGE + PADDING_SIZE,
			 				   angle, rx, ry);
			// int temp_points = extract_local_points(g_grid, GLOBAL_GRID_MAX_RANGE, l_points,
			// 						 LOCAL_GRID_MAX_RANGE + PADDING_SIZE, angle, rx, ry);

			float dangle, dx, dy, residual;
			// scan_to_map(l_points, temp_points, data_points, num_points, &dangle, &dx, &dy, &cost, &matches);
			 scan_to_map(l_grid, LOCAL_GRID_MAX_RANGE + PADDING_SIZE, data_points, num_points,
			 	 		&dangle, &dx, &dy, &residual);
			// // dangle *= 0.05;
			dx *= 0.05;
			dy *= 0.05;

			printf("update: %f %f %f %f \n", dx, dy,dangle, residual);

			if (1)
			{
				
			  	robot.Q += dangle;
				robot.pos.x += odom[0] ;
				robot.pos.y += odom[1] ;
			}
			else
			{
				robot.Q += odom[2];
				robot.pos.x += odom[0] ;
				robot.pos.y += odom[1] ;
			}
			
			// printf("residual: %f\n", residual);

			printf("odom: %f %f %f\n", odom[0], odom[1], odom[2]);

			while(robot.Q > 2* PI)
		 		robot.Q -= 2 * PI;
			while(robot.Q <  0)
		 		robot.Q += 2 * PI;

			update_map(g_grid, GLOBAL_GRID_MAX_RANGE, grid, LOCAL_GRID_MAX_RANGE,
					   robot.Q, roundf(robot.pos.x / 0.05), roundf(robot.pos.y / 0.05));

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

			float k_angle = 0.8;
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
			update_map(g_grid, GLOBAL_GRID_MAX_RANGE, grid, LOCAL_GRID_MAX_RANGE, robot.Q, (int)(robot.pos.x / 0.05), (int)(robot.pos.y / 0.05));                                         // Wait for a keystroke in the window
		}

		Mat lmap(LOCAL_GRID_PADDED_SIZE, LOCAL_GRID_PADDED_SIZE,CV_8UC1,l_grid);
		namedWindow( "Local map", WINDOW_AUTOSIZE );// Create a window for display.
   		Mat invlmap, szlmap;
   		bitwise_not ( lmap, invlmap ); //(, invgrid, 0, 255, CV_THRESH_BINARY_INV);
   		resize(invlmap, szlmap, cvSize(640, 480));
   		imshow( "Local map" , szlmap );                   // Show our image inside it.

   		Mat scan(LOCAL_GRID_SIZE, LOCAL_GRID_SIZE, CV_8UC1, grid);
		namedWindow( "Scan", WINDOW_AUTOSIZE );// Create a window for display.
   		Mat invscan, szscan;
   		bitwise_not ( scan, invscan ); //(, invgrid, 0, 255, CV_THRESH_BINARY_INV);
   		resize(invscan, szscan, cvSize(640, 480));
   		imshow( "Scan" , szscan );                   // Show our image inside it.

   		Mat gmap(GLOBAL_GRID_SIZE, GLOBAL_GRID_SIZE, CV_8UC1, g_grid);
		namedWindow( "Local map", WINDOW_AUTOSIZE );// Create a window for display.
   		Mat invgmap, szgmap;
   		bitwise_not ( gmap, invgmap ); //(, invgrid, 0, 255, CV_THRESH_BINARY_INV);
   		resize(invgmap, szgmap, cvSize(1280, 720));
   		imshow( "Global map" , szgmap );                   // Show our image inside it.
   		// Mat colorgrid;
   		// cv::cvtColor(invgrid, colorgrid, cv::COLOR_GRAY2BGR);

   		// // inverted axes for robot position
   		// int py = (int)(robot.pos.x / 0.05) + GLOBAL_GRID_MAX_RANGE;
   		// int px = (int)(robot.pos.y / 0.05) + GLOBAL_GRID_MAX_RANGE;
   		// circle(colorgrid, Point(px,py), 5, 0x00FFFF, -1);
  		waitKey(0); 	
	}

	//post_process_grid_erode(g_grid, pp_grid, GLOBAL_GRID_SIZE);

	dump_ocupancy_map(&g_grid[0][0], GLOBAL_GRID_MAX_RANGE, "grid.txt");

	cout << "Done!" << endl;
	int wait;
	cin >> wait;

	return 0;
}