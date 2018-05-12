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

		//evolve(&robot, time);
		// while(robot.Q > 2* PI)
		// 	robot.Q -= 2 * PI;
		// while(robot.Q <  0)
		// 	robot.Q += 2 * PI;

		float odom[3];
		getCurrentOdometry(&robot, time, odom);

		int num_points = convertLidarToSamplePoint(lidar[iter], data_points, num_lidar_samples);

		if (1)
		{
			int index = iter % 2;
			get_local_ocupancy_grid(data_points, num_points, grid, 0.1, LOCAL_GRID_MAX_RANGE);
			float qtest = 0.3, xtest = 0.08 / 0.1, ytest = -0.05 / 0.1;

			// Build template
			for(int i = 0 ; i < num_points ; ++i )
			{
				float x = roundf(data_points[i].x / 0.1);
				float y = roundf(data_points[i].y / 0.1);
				icp_template[2*i] = x;
				icp_template[2*i+1] = y;
			}
		    tp_num = num_points;

		    // Build model
		    float angle = robot.Q;
			int rx = roundf(robot.pos.x / 0.1);
			int ry = roundf(robot.pos.y / 0.1);

			// Extract all local points with high confidence
			num_points = extract_local_points(g_grid, GLOBAL_GRID_MAX_RANGE, l_points,
											  LOCAL_GRID_MAX_RANGE + PADDING_SIZE, angle, rx, ry);
			for(int i = 0 ; i < num_points ; ++i )
			{
				float x = l_points[i].x;
				float y = l_points[i].y;
				icp_model[2*i] = x;
				icp_model[2*i+1] = y;

			}
		    mp_num = num_points;

		    if(mp_num < 128) {

		    	robot.Q += odom[2];
	       		robot.pos.x += odom[0];
    			robot.pos.y += odom[1];

  				update_map(g_grid, GLOBAL_GRID_MAX_RANGE, grid, LOCAL_GRID_MAX_RANGE, robot.Q,
  						   roundf(robot.pos.x / 0.1), roundf(robot.pos.y / 0.1));
  				printf("state: %f %f %f\n", robot.pos.x, robot.pos.y, robot.Q );
  				continue;
  			}
  				
  			// Odometry accumulated since last lidar scan	
  			Matrix R(2,2);
			R.val[0][0] =  cos(odom[2]);
			R.val[0][1] =  -sin(odom[2]);
			R.val[1][0] =  sin(odom[2]);
			R.val[1][1] =  cos(odom[2]);

  			Matrix t(2,1);
  			t.val[0][0] =  (odom[0] / 0.1);
  			t.val[1][0] =  (odom[1] / 0.1);

  			printf("model size: %d   template size: %d\n", mp_num, tp_num );

	  		IcpPointToPlane icp(icp_model, mp_num, 2);
	        double residual = icp.fit(icp_template, tp_num, R, t, 0.1);

	           // cout << "R " << R << "\nt: " << t << endl; 
	         cout << t.val[0][0] << " " << t.val[1][0] << " " << atan2(R.val[1][0], R.val[0][0]) << 
	          			"  res: " << residual << endl;

	       // printf("inliers: %f\n", icp.getInlierCount());
	        if(1)
	        { 
	         	float nangle = atan2(R.val[1][0], R.val[0][0]);
	           	float nx = t.val[0][0] * 0.1;
	           	float ny = t.val[1][0] * 0.1;

				while(nangle >  PI)
					nangle -= 2 * PI;
				while(nangle <  -PI)
					nangle += 2 * PI;

	           	if(angle_diff(nangle,odom[2]) < 0.15 &&
	           		abs( nx - odom[0]) < 0.1 &&
	           		abs( ny - odom[1]) < 0.1 
	           		)
	           	{
	           		robot.Q += 1 * nangle + 0 * odom[2];
	           		robot.pos.x += 0 * nx + 1 * odom[0];
        			robot.pos.y += 0 * ny + 1 * odom[1];
       				printf("update!\n");
            	}
            	else
            	{
            		
	            	robot.Q += odom[2];
            		robot.pos.x += odom[0];
       				robot.pos.y += odom[1];
            	}

           		printf("deltas: \n %f vs %f\n %f vs %f\n %f vs %f\n",
       			odom[2] , nangle,
       			odom[0] , nx,
       			odom[1] , ny);	
			}
			else
	       	{
	       		robot.Q += odom[2];
	       		robot.pos.x += odom[0];
    			robot.pos.y += odom[1];
	       	}

			while(robot.Q > PI)
				robot.Q -= 2 * PI;
			while(robot.Q <  -PI)
				robot.Q += 2 * PI;

			printf("state: %f %f %f\n", robot.pos.x, robot.pos.y, robot.Q );
        	waitKey(0);

			// // start with identity as initial transformation
  	// 		// in practice you might want to use some kind of prediction here

			// int cost, matches;
			// scan_to_map(grid, LOCAL_GRID_MAX_RANGE, l_grid, LOCAL_GRID_MAX_RANGE + PADDING_SIZE, &dangle, &dx, &dy, &cost, &matches);
			// if (matches > 150)
			// {
			// 	printf("update: %d %d %d %d\n", dangle, dx, dy, matches);
			// 	robot.Q += dangle * DEGREES_TO_RADIAN;
			// 	robot.pos.x += dx * 0.1 * 0.5;
			// 	robot.pos.y += dy * 0.1 * 0.5;
			// }
			// printf("cost: %d %d %d %d %d\n", matches, cost, dx, dy, dangle);
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

		Mat mapgrid(GLOBAL_GRID_SIZE,GLOBAL_GRID_SIZE,CV_8UC1,g_grid);
		namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
   		 Mat invgrid;
   		 bitwise_not ( mapgrid, invgrid ); //(, invgrid, 0, 255, CV_THRESH_BINARY_INV);
   		// Mat colorgrid;
   		// cv::cvtColor(invgrid, colorgrid, cv::COLOR_GRAY2BGR);

   		// // inverted axes for robot position
   		// int py = (int)(robot.pos.x / 0.05) + GLOBAL_GRID_MAX_RANGE;
   		// int px = (int)(robot.pos.y / 0.05) + GLOBAL_GRID_MAX_RANGE;
   		// circle(colorgrid, Point(px,py), 5, 0x00FFFF, -1);
   		Mat newgrid;
   		resize(invgrid, newgrid, cvSize(1280, 720));

   	 	imshow( "Display window", newgrid );                   // Show our image inside it.

  		waitKey(1); 	
	}

	//post_process_grid_erode(g_grid, pp_grid, GLOBAL_GRID_SIZE);

	dump_ocupancy_map(&g_grid[0][0], GLOBAL_GRID_MAX_RANGE, "grid.txt");

	cout << "Done!" << endl;
	int wait;
	cin >> wait;

	return 0;
}