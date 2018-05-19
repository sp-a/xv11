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

void runOnDataset()
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
}

volatile extern uint8_t imu[16];
volatile extern float dists[360];
volatile extern uint64_t imu_ts;
uint64_t imu_prev_ts = 0;
float pvx = 0, pvy = 0;
int imu_count = 0;
void runWithSensors()
{
	int fd = openSerialPort("/dev/ttyACM2");
	int num_segments = 0;
	assert(fd >= 0);
	robot_state_t robot;

	robot.Q = 0;
	robot.pos.x = 0;
	robot.pos.y = 0;

	memset(g_grid, 128,  sizeof(g_grid));
	FeatureDetector detect(10, 30, (float)0.03, 50);
	
	while(1)
	{
		int type = readFromSerialPort(fd);
		if(type == 0)
		{
			// printf("lidar data available!\n");
			int num_points = 0;
			for(int i = 0; i < 360; ++i )
			{
				if(dists[i] == 0 || dists[i] > 6) continue;

				float angle  = (float)i * DEGREES_TO_RADIAN;
				float x = cos(angle) * dists[i];
				float y = sin(angle) * dists[i];

				data_points[num_points].x = x;
				data_points[num_points].y = y;
				num_points ++;
			}
			get_local_ocupancy_grid(data_points, num_points, grid, 0.05, LOCAL_GRID_MAX_RANGE);

			if(1)
			{
				num_segments = detect.extractSegmentsRansac(segments,
					data_points,
					num_points);
				num_segments = detect.filterSegments(segments, num_segments, PI / 3, 0.5);

				// Add new features
				for (int i = 0; i < num_segments; ++i)
					if (match_sg[i] == -1)
					{
						segments[i].age = 0;
						sg_db[num_sg_db++] = segments[i];
					}

				// printf("num points: %d\n", num_points);
				// printf("num segments: %d\n" , num_segments);

			}
			else
			{

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

				printf("                  update: %f %f %f %f \n", dx, dy,dangle, residual);

				if (1)
				{
					
				  	robot.Q += dangle;
					robot.pos.x += 0 ;
					robot.pos.y += 0 ;
				}
			

				while(robot.Q > 2* PI)
			 		robot.Q -= 2 * PI;
				while(robot.Q <  0)
			 		robot.Q += 2 * PI;
			}
			// memset(g_grid, 128, sizeof(g_grid));
			update_map(g_grid, GLOBAL_GRID_MAX_RANGE, grid, LOCAL_GRID_MAX_RANGE,
					   robot.Q, roundf(robot.pos.x / 0.05), roundf(robot.pos.y / 0.05));
                                       // Wait for a keystroke in the window

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
	   		Mat colorgrid;
	   		cv::cvtColor(invscan, colorgrid, cv::COLOR_GRAY2BGR);

	   		// {
	   		//  // inverted axes for robot position
	   		
	   		// int py = (int)(robot.pos.x / 0.05) + LOCAL_GRID_MAX_RANGE;
	   		// int px = (int)(robot.pos.y / 0.05) + LOCAL_GRID_MAX_RANGE;
	   		// circle(colorgrid, Point(px,py), 5, 0x00FFFF, -1);
	   		// }

	   		for(int i = 0 ; i < num_segments ; ++i )
	   		{
	   			int py0 = (int)(segments[i].edges[0].x / 0.05) + LOCAL_GRID_MAX_RANGE;
	   			int px0 = (int)(segments[i].edges[0].y / 0.05) + LOCAL_GRID_MAX_RANGE;
	   			int py1 = (int)(segments[i].edges[1].x / 0.05) + LOCAL_GRID_MAX_RANGE;
	   			int px1 = (int)(segments[i].edges[1].y / 0.05) + LOCAL_GRID_MAX_RANGE;
	   			line(colorgrid, Point(px0,py0), Point(px1,py1), 0x88, 2);
	   		}
	   		resize(colorgrid, szscan, cvSize(1280, 720));
	   		imshow( "Scan" , szscan );                   // Show our image inside it.

	   		Mat gmap(GLOBAL_GRID_SIZE, GLOBAL_GRID_SIZE, CV_8UC1, g_grid);
			namedWindow( "Local map", WINDOW_AUTOSIZE );// Create a window for display.
	   		Mat invgmap, szgmap;
	   		bitwise_not ( gmap, invgmap ); //(, invgrid, 0, 255, CV_THRESH_BINARY_INV);
	   		colorgrid;
	   		cv::cvtColor(invgmap, colorgrid, cv::COLOR_GRAY2BGR);
	   		{
	   		 // inverted axes for robot position
	   		int py = (int)(robot.pos.x / 0.05) + GLOBAL_GRID_MAX_RANGE;
	   		int px = (int)(robot.pos.y / 0.05) + GLOBAL_GRID_MAX_RANGE;
	   		circle(colorgrid, Point(px,py), 5, 0x00FFFF, -1);
	   		resize(colorgrid, szgmap, cvSize(1280, 720));
	   		}
	   		imshow( "Global map" , szgmap );                   // Show our image inside it.
	   		// Mat colorgrid;               // Show our image inside it.
	   		waitKey(1); 	
		}
		else
		{
			if(imu_prev_ts == 0)
				imu_prev_ts = imu_ts;

			imu_count ++;

			// printf("imu data available!\n");
  	   		int16_t ax = ((int16_t)imu[0] << 8) | imu[1];
	        float accx = (float) ax / 8192;
	        int16_t ay = ((int16_t)imu[2] << 8) | imu[3];
	        float accy = (float) ay / 8192;
	        int16_t az = ((int16_t)imu[4] << 8) | imu[5];
	        float accz = (float) az / 8192;
	        float acc = sqrt(accx*accx + accy*accy + accz*accz);

	        int16_t gx = ((int16_t)imu[6] << 8) | imu[7];
	        int16_t gy = ((int16_t)imu[8] << 8) | imu[9];
	        int16_t gz = ((int16_t)imu[10] << 8) | imu[11];

	        float  dt = (float)(imu_ts - imu_prev_ts) / 1000000; // sec
	        accx *= 9.80665; // m/s^2
	        accy *= 9.80665; // m/s^2
	        accz *= 9.80665; // m/s^2



	        float dq = (float)gz / 16.4 * 0.0174532925 * dt * 3; // radians
	        // printf("dq: %f\n", dq);
	        robot.Q += dq;
	        float nax = accx * cos(robot.Q) - accy * sin(robot.Q);
	        float nay = accy * sin(robot.Q) + accy * cos(robot.Q);

	        float vx = nax * dt;
	        float vy = nay * dt;

	        //if(abs(ax) > 200 || abs(ay) > 200)
	        {
	        	robot.pos.x += pvx * dt + 0.5 * nax * dt * dt;
	        	robot.pos.y += pvy * dt + 0.5 * nay * dt * dt;
	    	}

	        pvx = vx;
	        pvy = vy;

	        // for(int i = 0; i < 6 ; ++i )
	        // 	printf("0x%x%x ", imu[2*i], imu[2*i+1]);
	        // printf("\n");
	        // printf("ax: %d ay: %d az: %d gx: %d gy: %d gz: %d acc: %f delta %lu\n",
	        //  		ax, ay, az, gx, gy, gz, acc, imu_ts - imu_prev_ts);
	        imu_prev_ts = imu_ts;
		}

		// printf("---------------------- x %f y %f q %f\n" , robot.pos.x, robot.pos.y, robot.Q * 57.2957795);
	}
}

int main()
{
	runWithSensors();

	return 0;
}