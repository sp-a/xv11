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
uint8_t v_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE];
uint8_t l_grid[LOCAL_GRID_PADDED_SIZE][LOCAL_GRID_PADDED_SIZE];
uint8_t pp_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE];

point_t l_points[LOCAL_GRID_SIZE*LOCAL_GRID_SIZE];
double icp_template[MAX_LIDAR_SAMPLES * 2];
double icp_model[LOCAL_GRID_PADDED_SIZE * LOCAL_GRID_PADDED_SIZE];
int tp_num, mp_num;

int rstatex[1024*8];
int rstatey[1024*8];
int num_states = 0;

float map_resolution = 0.05;


double angle_diff(double a, double b)
{
	double q1 = min(a,b);
	double q2 = max(a,b);

	return min(abs(q1-q2), abs(q1 + 2 * PI -q2));
}

void runOnDataset()
{
	FeatureDetector detect(10, 30, (float)0.005, 50);

	int num_odometry_samples = readOdometryFromFile(odometry, MAX_ODOMETRY_SAMPLES);
	int num_lidar_samples    = readLidarFromFile(lidar, MAX_LIDAR_SAMPLES);
	clearFile("Logs/state.txt");

	init_trig_lut();

	robot_state_t robot;
	initializeOdometry(&robot, odometry, num_odometry_samples);

	for(int i = 0; i < GLOBAL_GRID_SIZE ; ++i )
		for(int j = 0; j < GLOBAL_GRID_SIZE; ++j)
			g_grid[i][j] = v_grid[i][j] = 128; // unknown


	for (int iter = 0; iter < num_lidar_samples; ++iter) {

		float time = lidar[iter].ts; // current time
		int dangle, dx, dy;

		evolve(&robot, time);
		while(robot.Q > 2* PI)
		  	robot.Q -= 2 * PI;
		while(robot.Q <  0)
		  	robot.Q += 2 * PI;

		float odom[3];
		// getCurrentOdometry(&robot, time, odom);
		int num_points = convertLidarToSamplePoint(lidar[iter], data_points, num_lidar_samples);
		int num_segments = 0;

		//if (1)
		{
			int index = iter % 2;
			get_local_ocupancy_grid(data_points, num_points, grid, map_resolution, LOCAL_GRID_MAX_RANGE);

			num_segments = detect.extractSegmentsRansac(segments, data_points, num_points);
			printf("num segments: %d\n", num_segments);

			for(int i = 0 ; i < num_points ; ++i )
			{
				data_points[i].x = roundf(data_points[i].x / map_resolution);
				data_points[i].y = roundf(data_points[i].y /map_resolution);
			}

			float angle = robot.Q;
			int rx = (int)roundf(robot.pos.x / map_resolution);
			int ry = (int)roundf(robot.pos.y / map_resolution);
			extract_local_grid(g_grid, GLOBAL_GRID_MAX_RANGE, l_grid, LOCAL_GRID_MAX_RANGE + PADDING_SIZE,
			 				   angle, rx, ry);

			float dangle, dx, dy, residual;
			scan_to_map(l_grid, LOCAL_GRID_MAX_RANGE + PADDING_SIZE, data_points, num_points,
			   	 		&dangle, &dx, &dy, &residual);

			dx *= map_resolution;
			dy *= map_resolution;
			printf("update: %f %f %f %f \n", dx, dy,dangle, residual);

			robot.Q += dangle;
			// robot.pos.x += odom[0] ;
			// robot.pos.y += odom[1] ;

			while(robot.Q > 2* PI)
		 		robot.Q -= 2 * PI;
			while(robot.Q <  0)
		 		robot.Q += 2 * PI;

			for(int i = 0 ; i < num_segments; ++i  )
			{
				match_sg[i] = -1;
				segment_t temp_sg = segments[i];
				transform_segment(&temp_sg, robot.pos.x, robot.pos.y, robot.Q);
				if(temp_sg.slope < 0)
						temp_sg.slope += PI;

				int best_cost = 1000000;

				for(int j = 0 ; j < num_sg_db; ++ j)
				{
					float mdist = distance(temp_sg.middle, sg_db[j].middle);
					float pdist = abs(sg_db[j].m * temp_sg.middle.x - temp_sg.middle.y + sg_db[j].n) / 
									sqrt(sg_db[j].m*sg_db[j].m + 1);
					float ldif = abs(sg_length(temp_sg) - sg_length(sg_db[j]));
					if(slope_dif(temp_sg.slope, sg_db[j].slope) < PI / 36 && pdist < 0.6 && mdist < 1.0)
					{
						if(best_cost > mdist * 10 + pdist)
						{

							match_sg[i] = j;
							best_cost = mdist * 10 + pdist;
						}
					}
				}

				if(match_sg[i] != -1)
				{
					sg_db[match_sg[i]].seen ++;
					float slope0 = temp_sg.slope;
					float slope1 = sg_db[match_sg[i]].slope;
					float dq;

					{

					if(slope1 > slope0)
					{
						if(slope1 - slope0 < slope0 + PI - slope1)
							dq = slope1 - slope0;
						else
							dq = slope1 - slope0 - PI;
					}
					else
					{
						if(slope0 - slope1 < slope1 + PI - slope0)
							dq = slope1 - slope0; // negativ
						else
							dq = slope0 - slope1 - PI;
					}

					printf("update db: %f sg: %f dq: %f\n",sg_db[match_sg[i]].slope, temp_sg.slope, dq);
					robot.Q += dq;

					segment_t sg0 = segments[i];
					transform_segment(&sg0, robot.pos.x, robot.pos.y, robot.Q);
					segment_t sg1 = sg_db[match_sg[i]];
					
					float m = -1 / sg1.m;
					float n = sg0.middle.y - m * sg0.middle.x;

					// y2 = m * x2 + n
					// y2 = sg1.m * x2 + sg1.n
					float x2 = -(sg1.n - n) / (sg1.m - m);
					float y2 = m * x2 + n;


					float dx = x2 - sg0.middle.x;
					float dy = y2 - sg0.middle.y;

					robot.pos.x += dx;
					robot.pos.y += dy;

					}
				}
			}

			rstatex[num_states] = (int)roundf(robot.pos.x / map_resolution) + GLOBAL_GRID_MAX_RANGE;
			rstatey[num_states] = (int)roundf(robot.pos.y / map_resolution) + GLOBAL_GRID_MAX_RANGE;
			dumpStateToFile(&robot, "state.txt", iter);
			num_states ++;

			// remove old segments
			// int count = 0;
			// for(int i = 0 ; i < num_sg_db; ++i  )
			// {
			// 	if(!(sg_db[i].seen < 5 && ++sg_db[i].age > 20))
			// 		sg_db[count++] = sg_db[i];
			// }
			// num_sg_db = count;

			printf("update segment database:\n");
			// Add new features
			for (int i = 0; i < num_segments; ++i)
				if (match_sg[i] == -1)
				{
					segment_t temp_sg = segments[i];
					transform_segment(&temp_sg, robot.pos.x, robot.pos.y, robot.Q);
					temp_sg.age = 0;
					temp_sg.seen = 1;
					if(temp_sg.slope < 0)
						temp_sg.slope += PI;
					int good = 1;
					for(int j = 0; j < num_sg_db ; ++j )
					{
						float mdist = distance(temp_sg.middle, sg_db[j].middle);
						float pdist = abs(sg_db[j].m * temp_sg.middle.x - temp_sg.middle.y + sg_db[j].n) / 
										sqrt(sg_db[j].m*sg_db[j].m + 1);
						if(mdist < 1 || pdist < 1)
						{
							good = 0;
							if(sg_length(temp_sg) > sg_length(sg_db[j]))
							{
								sg_db[j] = temp_sg;
							}
							break;
						}
					}
					sg_db[num_sg_db++] = temp_sg;
				}

			// memset(g_grid, 128, sizeof(g_grid));
			update_map(g_grid, GLOBAL_GRID_MAX_RANGE, grid, LOCAL_GRID_MAX_RANGE,
			 		   robot.Q, roundf(robot.pos.x / map_resolution), roundf(robot.pos.y / map_resolution), 0);
			update_map(v_grid, GLOBAL_GRID_MAX_RANGE, grid, LOCAL_GRID_MAX_RANGE,
			 		   robot.Q, roundf(robot.pos.x / map_resolution), roundf(robot.pos.y / map_resolution), 1);
		}

		Mat lmap(LOCAL_GRID_PADDED_SIZE, LOCAL_GRID_PADDED_SIZE, CV_8UC1 ,l_grid);
		namedWindow( "Local map", WINDOW_AUTOSIZE );// Create a window for display.
   		Mat invlmap, szlmap;
   		bitwise_not ( lmap, invlmap ); //(, invgrid, 0, 255, CV_THRESH_BINARY_INV);
   		resize(invlmap, szlmap, cvSize(640, 480));
   		Mat fliplocal, hfliplocal;
   		flip(szlmap, fliplocal, 0);
   		flip(fliplocal, hfliplocal, 1);
   		//imshow( "Local map" , fliplocal );                   // Show our image inside it.

   		Mat lmmap(GLOBAL_GRID_SIZE, GLOBAL_GRID_SIZE, CV_8UC1, 150);
   		namedWindow( "Landmarks", WINDOW_AUTOSIZE );// Create a window for display.
   		Mat colorlm, szlm;
	   	cv::cvtColor(lmmap, colorlm, cv::COLOR_GRAY2BGR);
	   	for(int i = 0 ; i < num_sg_db ; ++i )
	   	{
	   		int py0 = (int)(sg_db[i].edges[0].x / map_resolution) + GLOBAL_GRID_MAX_RANGE;
	   		int px0 = (int)(sg_db[i].edges[0].y / map_resolution) + GLOBAL_GRID_MAX_RANGE;
	   		int py1 = (int)(sg_db[i].edges[1].x / map_resolution) + GLOBAL_GRID_MAX_RANGE;
	   		int px1 = (int)(sg_db[i].edges[1].y / map_resolution) + GLOBAL_GRID_MAX_RANGE;
	   		if(sg_db[i].seen < 10)
	   			line(colorlm, Point(px0,py0), Point(px1,py1), CV_RGB(0, 128, 0), 4);
	   		else
	   			line(colorlm, Point(px0,py0), Point(px1,py1), CV_RGB(128, 0, 0), 4);
	   	}
	   	resize(colorlm, szlm, cvSize(640, 480));
	   	imshow( "Landmarks" , szlm ); 
	   	moveWindow("Landmarks", 20,500);

   		Mat scan(LOCAL_GRID_SIZE, LOCAL_GRID_SIZE, CV_8UC1, grid);
		namedWindow( "Scan", WINDOW_AUTOSIZE );// Create a window for display.
	   	Mat invscan, szscan;
	   	bitwise_not ( scan, invscan ); //(, invgrid, 0, 255, CV_THRESH_BINARY_INV);
	   	Mat colorgrid;
	   	cv::cvtColor(invscan, colorgrid, cv::COLOR_GRAY2BGR);

	   	for(int i = 0 ; i < num_segments ; ++i )
	   	{
	   		int py0 = (int)(segments[i].edges[0].x / map_resolution) + LOCAL_GRID_MAX_RANGE;
	   		int px0 = (int)(segments[i].edges[0].y / map_resolution) + LOCAL_GRID_MAX_RANGE;
	   		int py1 = (int)(segments[i].edges[1].x / map_resolution) + LOCAL_GRID_MAX_RANGE;
	   		int px1 = (int)(segments[i].edges[1].y / map_resolution) + LOCAL_GRID_MAX_RANGE;
	   		if(match_sg[i] == -1)
	   			line(colorgrid, Point(px0,py0), Point(px1,py1), CV_RGB(0, 128, 0), 2);	   			
	   		else
	   			line(colorgrid, Point(px0,py0), Point(px1,py1), CV_RGB(128, 0, 0), 2);
	   	}
	   	circle(colorgrid, Point(LOCAL_GRID_MAX_RANGE, LOCAL_GRID_MAX_RANGE), 8, CV_RGB(50, 50, 50), -1);
		line(colorgrid, Point(LOCAL_GRID_MAX_RANGE, LOCAL_GRID_MAX_RANGE), Point(LOCAL_GRID_MAX_RANGE, LOCAL_GRID_MAX_RANGE+8),  CV_RGB(0, 0, 0), 2);
	   	resize(colorgrid, szscan, cvSize(640, 480));
	   	Mat flipscan, hflipscan;
	   	flip(szscan, flipscan,0);
	   	flip(flipscan, hflipscan,0);
	   	imshow( "Scan" , flipscan );                   // Show our image inside it.
	   	moveWindow("Scan", 20,20);

   		Mat gmap(GLOBAL_GRID_SIZE, GLOBAL_GRID_SIZE, CV_8UC1, v_grid);
		namedWindow( "Local map", WINDOW_AUTOSIZE );// Create a window for display.
   		Mat invgmap, szgmap;
   		bitwise_not ( gmap, invgmap ); //(, invgrid, 0, 255, CV_THRESH_BINARY_INV);
   		cv::cvtColor(invgmap, colorgrid, cv::COLOR_GRAY2BGR);
   		for(int i = 0; i < num_states; ++i)
   		{
   			circle(colorgrid, Point(rstatey[i], rstatex[i]), 2, 0x00FFFF, -1);
   		}
   		// inverted axes for robot position
   		{
	   		int py = (int)(robot.pos.x / map_resolution) + GLOBAL_GRID_MAX_RANGE;
	   		int px = (int)(robot.pos.y / map_resolution) + GLOBAL_GRID_MAX_RANGE;
	   		circle(colorgrid, Point(px,py), 8, 0x00FFFF, -1);
   		}
   		resize(colorgrid, szgmap, cvSize(1280, 720));
   		imshow( "Global map" , szgmap );                   // Show our image inside it.
   		moveWindow("Global map", 640,20);
  		waitKey(1); 	
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
float pvx = 0, pvy = 0, prev_fy = 0;
int imu_count = 0;

class Kalman { 
 public:
    float _q; //process noise covariance
    float _r; //measurement noise covariance
    float _x; //value
    float _p; //estimation error covariance
    float _k; //kalman gain  

    Kalman (float q, float r, float p, float intial_value) 
    { // The Constructor is defined with arguments.
      _q=q;
      _r=r;
      _p=p;
      _x=intial_value;
    }

   float update(float measurement)
   {
    //prediction update
  //omit x = x
  _p = _p +_q;
  //measurement update
  _k = _p / (_p + _r);
  _x = _x + _k * (measurement - _x);
  _p = (1.0 -_k) * _p;
  return _x;
  }
};

Kalman klmx(0.1,0.001,0.001,0.0000001);
Kalman klmy(0.1,0.001,0.001,0.0000001);
Kalman klmz(0.1,0.001,0.001,0.0000001);

void runWithSensors()
{
	// sleep(20);
	int fd = openSerialPort("/dev/ttyACM0");
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
				// if(dists[i] == 0 || dists[i] > 6) continue;

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

				segment_t tem_sg[32];
				for (int i = 0; i < num_segments; ++i)
					tem_sg[i] = segments[i];
				transform_segments(tem_sg, num_segments, robot.pos.x, robot.pos.y, robot.Q);
				match_segments(tem_sg, num_segments, sg_db, num_sg_db, match_sg, inn_sg, robot);

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

					printf("update angle: %f\n", k_angle * sumq );
				}
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

				// printf("                  update: %f %f %f %f \n", dx, dy,dangle, residual);

				if (0)
				{
					
				  	robot.Q += dangle;
					robot.pos.x += dx ;
					robot.pos.y += dy ;
				}
			

				while(robot.Q > 2* PI)
			 		robot.Q -= 2 * PI;
				while(robot.Q <  0)
			 		robot.Q += 2 * PI;
			}
			// memset(g_grid, 128, sizeof(g_grid));
			update_map(g_grid, GLOBAL_GRID_MAX_RANGE, grid, LOCAL_GRID_MAX_RANGE,
					   robot.Q, roundf(robot.pos.x / 0.05), roundf(robot.pos.y / 0.05),  0);
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
  	   		int16_t ax = ((int16_t)imu[1] << 8) | imu[0];
	        float accx = (float) ax / 8192;
	        int16_t ay = ((int16_t)imu[3] << 8) | imu[2];
	        float accy = (float) ay / 8192;
	        int16_t az = ((int16_t)imu[5] << 8) | imu[4];
	        float accz = (float) az / 8192;
	        
	    //     float ac[3];
  			// ac[0] = klmx.update(accx) * 9.80665;
  			// ac[1] = klmy.update(accy) * 9.80665;
  			// ac[2] = klmz.update(accz) * 9.80665;

	    //     int16_t iy = ((int16_t)imu[7] << 8) | imu[6];
	    //     int16_t ip = ((int16_t)imu[9] << 8) | imu[8];
	    //     int16_t ir = ((int16_t)imu[11] << 8) | imu[10];

	    //     float fy = (float)iy / 100;
	    //     float fp = (float)ip / 100;
	    //     float fr = (float)ir / 100;

	    //     float  dt = (float)(imu_ts - imu_prev_ts) / 1000000; // sec

	    //     robot.Q += fy - prev_fy;
	    //     prev_fy = fy;
	        
	    //     float dx =  0.5 * ac[0] * dt * dt * 18;
	    //     float dy =  0.5 * ac[1] * dt * dt * 18;
	        
	    //     robot.pos.x += cos(robot.Q) * dx - sin(robot.Q) * dy;
	    //     robot.pos.y += sin(robot.Q) * dx + cos(robot.Q) * dy;

	    //     printf("%f %f %.2f dt %.2f\n", pvx, pvy, robot.Q, dt);

	        imu_prev_ts = imu_ts;
		}

		// printf("---------------------- x %f y %f q %f\n" , robot.pos.x, robot.pos.y, robot.Q * 57.2957795);
	}
}

int lidar_num = 0;
float lidar_data[1024*8][360];
uint64_t lidar_data_ts[1024*8];
int imu_num = 0;
uint8_t imu_data[1024*8][12];
uint64_t imu_data_ts[1024*8];
int curr_lidar  = 0, curr_imu = 0;
void runwithSavedData()
{
	robot_state_t robot;

	robot.pos.x = 0;
	robot.pos.y = 0;
	robot.Q = 0;
	FeatureDetector detect(50, 10, (float)0.005, 200);

	FILE *fl = fopen("Captures/lidar_dataset.txt", "r");
	while(!feof(fl))
	{
		for(int i = 0 ; i < 360; ++i )
			fscanf(fl, "%f", &lidar_data[lidar_num][i]);
		fscanf(fl, "%lu", &lidar_data_ts[lidar_num]);
		lidar_num ++;
	}
	fclose(fl);
	printf("lidar samples: %d\n", lidar_num);

	FILE *fi = fopen("Captures/imu_dataset.txt", "r");
	while(!feof(fi))
	{
		for(int i = 0 ; i < 12 ; ++i )
			fscanf(fi, "%hhu", &imu_data[imu_num][i]);
		fscanf(fi, "%lu", &imu_data_ts[imu_num]);
		imu_num ++; 

	}
	fclose(fi);
	printf("imu samples: %d\n", imu_num);
	float fdx= 0, fdy = 0, fdq = 0;
	memset(g_grid, 128, sizeof(g_grid));
	memset(v_grid, 128, sizeof(v_grid));

	while(curr_imu < imu_num && curr_lidar < lidar_num)
	{
		if(imu_data_ts[curr_imu] < lidar_data_ts[curr_lidar])
		{

			printf("new imu sample, ts %lu!\n", imu_data_ts[curr_imu]);
	        int16_t gz = ((int16_t)imu_data[curr_imu][10] << 8) | imu_data[curr_imu][11];
	        float dt;
	        if(curr_imu == 0) dt = 0;
	        else dt = (float)(imu_data_ts[curr_imu] - imu_data_ts[curr_imu-1]) / 1000000; // sec

	       if(imu_prev_ts == 0)
				imu_prev_ts = imu_ts;

			// printf("imu data available!\n");
  	   		int16_t ax = ((int16_t)imu_data[curr_imu][1] << 8) | imu_data[curr_imu][0];
	        float accx = (float) ax / 8192;
	        int16_t ay = ((int16_t)imu_data[curr_imu][3] << 8) | imu_data[curr_imu][2];
	        float accy = (float) ay / 8192;
	        int16_t az = ((int16_t)imu_data[curr_imu][5] << 8) | imu_data[curr_imu][4];
	        float accz = (float) az / 8192;
	        
	        float ac[3];
  			ac[0] = klmx.update(accx) * 9.80665;
  			ac[1] = klmy.update(accy) * 9.80665;
  			ac[2] = klmz.update(accz) * 9.80665;

	        int16_t iy = ((int16_t)imu_data[curr_imu][7] << 8) | imu_data[curr_imu][6];
	        int16_t ip = ((int16_t)imu_data[curr_imu][9] << 8) | imu_data[curr_imu][8];
	        int16_t ir = ((int16_t)imu_data[curr_imu][11] << 8) | imu_data[curr_imu][10];

	        float fy = (float)iy / 100;
	        float fp = (float)ip / 100;
	        float fr = (float)ir / 100;

	        robot.Q -= fy - prev_fy;

	        printf("imu dq: %.2f " , fy - prev_fy);
	        prev_fy = fy;
	        
	        float dx =  0.5 * ac[0] * dt * dt * 18;
	        float dy =  0.5 * ac[1] * dt * dt * 18;
	        
	        float rdx = cos(robot.Q) * dx - sin(robot.Q) * dy;
	        float rdy = sin(robot.Q) * dx + cos(robot.Q) * dy;

	        robot.pos.x -= 0.01 * cos(robot.Q);
	        robot.pos.y -= 0.01 * sin(robot.Q);

	        printf("dx %.2f  dy  %.2f\n", rdx, rdy);

	        imu_prev_ts = imu_ts;

			curr_imu ++;
		}
		else
		{
			printf("new lidar sample!\n");
			int num_points = 0;
			int num_segments = 0;	

			for(int i = 0; i < 360; ++i )
			{
				if((int)(lidar_data[curr_lidar][i] * 1000) & 0xC000) continue;
				if(lidar_data[curr_lidar][i] > 6.0) continue;

				float angle  = (float)i * DEGREES_TO_RADIAN;
				float x = cos(angle) * lidar_data[curr_lidar][i];
				float y = sin(angle) * lidar_data[curr_lidar][i];

				if(sqrt(x*x + y*y) < 1.0) continue;

				data_points[num_points].x = x;
				data_points[num_points].y = y;
				num_points ++;
			}

			{

			get_local_ocupancy_grid(data_points, num_points, grid, map_resolution, LOCAL_GRID_MAX_RANGE);

			num_segments = detect.extractSegmentsRansac(segments, data_points, num_points);
			// printf("num segments: %d\n", num_segments);

			for(int i = 0 ; i < num_points ; ++i )
			{
				data_points[i].x = roundf(data_points[i].x / map_resolution);
				data_points[i].y = roundf(data_points[i].y /map_resolution);
			}

			float angle = robot.Q;
			int rx = (int)roundf(robot.pos.x / map_resolution);
			int ry = (int)roundf(robot.pos.y / map_resolution);
			extract_local_grid(g_grid, GLOBAL_GRID_MAX_RANGE, l_grid, LOCAL_GRID_MAX_RANGE + PADDING_SIZE,
			 				   angle, rx, ry);

			float dangle, dx, dy, residual;
			// scan_to_map(l_grid, LOCAL_GRID_MAX_RANGE + PADDING_SIZE, data_points, num_points,
			//   	 		&dangle, &dx, &dy, &residual);

			dx *= map_resolution;
			dy *= map_resolution;
			printf("update: %f %f %f %f \n", dx, dy,dangle, residual);

			robot.Q += dangle;
			robot.pos.x += dx ;
			robot.pos.y += dy ;

			while(robot.Q > 2* PI)
		 		robot.Q -= 2 * PI;
			while(robot.Q <  0)
		 		robot.Q += 2 * PI;

			for(int i = 0 ; i < num_segments; ++i  )
			{
				match_sg[i] = -1;
				segment_t temp_sg = segments[i];
				transform_segment(&temp_sg, robot.pos.x, robot.pos.y, robot.Q);
				if(temp_sg.slope < 0)
						temp_sg.slope += PI;

				int best_cost = 1000000;

				for(int j = 0 ; j < num_sg_db; ++ j)
				{
					float mdist = distance(temp_sg.middle, sg_db[j].middle);
					float pdist = abs(sg_db[j].m * temp_sg.middle.x - temp_sg.middle.y + sg_db[j].n) / 
									sqrt(sg_db[j].m*sg_db[j].m + 1);
					float ldif = abs(sg_length(temp_sg) - sg_length(sg_db[j]));
					if(slope_dif(temp_sg.slope, sg_db[j].slope) < PI / 36 && pdist < 0.6 && mdist < 1.0)
					{
						if(best_cost > mdist * 10 + pdist)
						{

							match_sg[i] = j;
							best_cost = mdist * 10 + pdist;
						}
					}
				}

				if(match_sg[i] != -1)
				{
					sg_db[match_sg[i]].seen ++;
					float slope0 = temp_sg.slope;
					float slope1 = sg_db[match_sg[i]].slope;
					float dq;

					{

					if(slope1 > slope0)
					{
						if(slope1 - slope0 < slope0 + PI - slope1)
							dq = slope1 - slope0;
						else
							dq = slope1 - slope0 - PI;
					}
					else
					{
						if(slope0 - slope1 < slope1 + PI - slope0)
							dq = slope1 - slope0; // negativ
						else
							dq = slope0 - slope1 - PI;
					}

					printf("update db: %f sg: %f dq: %f\n",sg_db[match_sg[i]].slope, temp_sg.slope, dq);
					robot.Q += dq;

					segment_t sg0 = segments[i];
					transform_segment(&sg0, robot.pos.x, robot.pos.y, robot.Q);
					segment_t sg1 = sg_db[match_sg[i]];
					
					float m = -1 / sg1.m;
					float n = sg0.middle.y - m * sg0.middle.x;

					// y2 = m * x2 + n
					// y2 = sg1.m * x2 + sg1.n
					float x2 = -(sg1.n - n) / (sg1.m - m);
					float y2 = m * x2 + n;


					float dx = x2 - sg0.middle.x;
					float dy = y2 - sg0.middle.y;

					robot.pos.x += dx;
					robot.pos.y += dy;

					}
				}
			}

			rstatex[num_states] = (int)roundf(robot.pos.x / map_resolution) + GLOBAL_GRID_MAX_RANGE;
			rstatey[num_states] = (int)roundf(robot.pos.y / map_resolution) + GLOBAL_GRID_MAX_RANGE;
			num_states ++;

			// remove old segments
			// int count = 0;
			// for(int i = 0 ; i < num_sg_db; ++i  )
			// {
			// 	if(!(sg_db[i].seen < 5 && ++sg_db[i].age > 20))
			// 		sg_db[count++] = sg_db[i];
			// }
			// num_sg_db = count;

			printf("update segment database:\n");
			// Add new features
			for (int i = 0; i < num_segments; ++i)
				if (match_sg[i] == -1)
				{
					segment_t temp_sg = segments[i];
					transform_segment(&temp_sg, robot.pos.x, robot.pos.y, robot.Q);
					temp_sg.age = 0;
					temp_sg.seen = 1;
					if(temp_sg.slope < 0)
						temp_sg.slope += PI;
					int good = 1;
					for(int j = 0; j < num_sg_db ; ++j )
					{
						float mdist = distance(temp_sg.middle, sg_db[j].middle);
						float pdist = abs(sg_db[j].m * temp_sg.middle.x - temp_sg.middle.y + sg_db[j].n) / 
										sqrt(sg_db[j].m*sg_db[j].m + 1);
						if(mdist < 1 || pdist < 1)
						{
							good = 0;
							if(sg_length(temp_sg) > sg_length(sg_db[j]))
							{
								sg_db[j] = temp_sg;
							}
							break;
						}
					}
					sg_db[num_sg_db++] = temp_sg;
				}

			// memset(g_grid, 128, sizeof(g_grid));
			update_map(g_grid, GLOBAL_GRID_MAX_RANGE, grid, LOCAL_GRID_MAX_RANGE,
			 		   robot.Q, roundf(robot.pos.x / map_resolution), roundf(robot.pos.y / map_resolution), 0);
			update_map(v_grid, GLOBAL_GRID_MAX_RANGE, grid, LOCAL_GRID_MAX_RANGE,
			 		   robot.Q, roundf(robot.pos.x / map_resolution), roundf(robot.pos.y / map_resolution), 1);
		}

		Mat lmap(LOCAL_GRID_PADDED_SIZE, LOCAL_GRID_PADDED_SIZE, CV_8UC1 ,l_grid);
		namedWindow( "Local map", WINDOW_AUTOSIZE );// Create a window for display.
   		Mat invlmap, szlmap;
   		bitwise_not ( lmap, invlmap ); //(, invgrid, 0, 255, CV_THRESH_BINARY_INV);
   		resize(invlmap, szlmap, cvSize(640, 480));
   		Mat fliplocal, hfliplocal;
   		flip(szlmap, fliplocal, 0);
   		flip(fliplocal, hfliplocal, 1);
   		//imshow( "Local map" , fliplocal );                   // Show our image inside it.

   		Mat lmmap(GLOBAL_GRID_SIZE, GLOBAL_GRID_SIZE, CV_8UC1, 150);
   		namedWindow( "Landmarks", WINDOW_AUTOSIZE );// Create a window for display.
   		Mat colorlm, szlm;
	   	cv::cvtColor(lmmap, colorlm, cv::COLOR_GRAY2BGR);
	   	for(int i = 0 ; i < num_sg_db ; ++i )
	   	{
	   		int py0 = (int)(sg_db[i].edges[0].x / map_resolution) + GLOBAL_GRID_MAX_RANGE;
	   		int px0 = (int)(sg_db[i].edges[0].y / map_resolution) + GLOBAL_GRID_MAX_RANGE;
	   		int py1 = (int)(sg_db[i].edges[1].x / map_resolution) + GLOBAL_GRID_MAX_RANGE;
	   		int px1 = (int)(sg_db[i].edges[1].y / map_resolution) + GLOBAL_GRID_MAX_RANGE;
	   		if(sg_db[i].seen < 10)
	   			line(colorlm, Point(px0,py0), Point(px1,py1), CV_RGB(0, 128, 0), 4);
	   		else
	   			line(colorlm, Point(px0,py0), Point(px1,py1), CV_RGB(128, 0, 0), 4);
	   	}
	   	resize(colorlm, szlm, cvSize(640, 480));
	   	imshow( "Landmarks" , szlm ); 
	   	moveWindow("Landmarks", 20,500);

   		Mat scan(LOCAL_GRID_SIZE, LOCAL_GRID_SIZE, CV_8UC1, grid);
		namedWindow( "Scan", WINDOW_AUTOSIZE );// Create a window for display.
	   	Mat invscan, szscan;
	   	bitwise_not ( scan, invscan ); //(, invgrid, 0, 255, CV_THRESH_BINARY_INV);
	   	Mat colorgrid;
	   	cv::cvtColor(invscan, colorgrid, cv::COLOR_GRAY2BGR);

	   	for(int i = 0 ; i < num_segments ; ++i )
	   	{
	   		int py0 = (int)(segments[i].edges[0].x / map_resolution) + LOCAL_GRID_MAX_RANGE;
	   		int px0 = (int)(segments[i].edges[0].y / map_resolution) + LOCAL_GRID_MAX_RANGE;
	   		int py1 = (int)(segments[i].edges[1].x / map_resolution) + LOCAL_GRID_MAX_RANGE;
	   		int px1 = (int)(segments[i].edges[1].y / map_resolution) + LOCAL_GRID_MAX_RANGE;
	   		if(match_sg[i] == -1)
	   			line(colorgrid, Point(px0,py0), Point(px1,py1), CV_RGB(0, 128, 0), 2);	   			
	   		else
	   			line(colorgrid, Point(px0,py0), Point(px1,py1), CV_RGB(128, 0, 0), 2);
	   	}
	   	circle(colorgrid, Point(LOCAL_GRID_MAX_RANGE, LOCAL_GRID_MAX_RANGE), 2, CV_RGB(50, 50, 50), -1);
		line(colorgrid, Point(LOCAL_GRID_MAX_RANGE, LOCAL_GRID_MAX_RANGE), Point(LOCAL_GRID_MAX_RANGE, LOCAL_GRID_MAX_RANGE+2),  CV_RGB(0, 0, 0), 2);
	   	resize(colorgrid, szscan, cvSize(640, 480));
	   	Mat flipscan, hflipscan;
	   	//flip(szscan, flipscan,0);
	   	//flip(flipscan, hflipscan,0);
	   	imshow( "Scan" , szscan );                   // Show our image inside it.
	   	moveWindow("Scan", 20,20);

   		Mat gmap(GLOBAL_GRID_SIZE, GLOBAL_GRID_SIZE, CV_8UC1, v_grid);
		namedWindow( "Local map", WINDOW_AUTOSIZE );// Create a window for display.
   		Mat invgmap, szgmap;
   		bitwise_not ( gmap, invgmap ); //(, invgrid, 0, 255, CV_THRESH_BINARY_INV);
   		cv::cvtColor(invgmap, colorgrid, cv::COLOR_GRAY2BGR);
   		for(int i = 0; i < num_states; ++i)
   		{
   			circle(colorgrid, Point(rstatey[i], rstatex[i]), 1, 0x00FFFF, -1);
   		}
   		// inverted axes for robot position
   		{
	   		int py = (int)(robot.pos.x / map_resolution) + GLOBAL_GRID_MAX_RANGE;
	   		int px = (int)(robot.pos.y / map_resolution) + GLOBAL_GRID_MAX_RANGE;
	   		circle(colorgrid, Point(px,py), 3, 0x00FFFF, -1);
   		}
   		resize(colorgrid, szgmap, cvSize(1280, 720));
   		imshow( "Global map" , szgmap );                   // Show our image inside it.
   		moveWindow("Global map", 640,20);
  		waitKey(1); 	

	   	curr_lidar ++;
		}
	}
	
	cout << "Done!" << endl;
	int wait;
	cin >> wait;
}

int main()
{
	clearFile("Captures/lidar.txt");
	clearFile("Captures/imu.txt");
	// runWithSensors();
	// runOnDataset();

	runwithSavedData();

	return 0;
}