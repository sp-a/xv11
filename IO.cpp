#include "IO.h"

static const char* logFile = "dataset.log";

#define PI 3.14159265359

int readOdometryFromFile(odometry_t *odometry, int max_num_readings)
{
	char line[16 * 1024];
	odometry_t odo;
	float dummy;
	char dummyN[10];
	int index = 0;

	FILE *f = fopen(logFile, "r");
	if (f == NULL)
		return -1;

	while (!feof(f))
	{
		fgets(line, 16 * 1024, f);
		if (strstr(line, "ODOM") == line) // Find 'ODOM' at begining
		{
			// ODOM x y theta tv rv accel
			sscanf(line, "ODOM %f %f %f %f %f %f %f %s %f", &odo.x, &odo.y, &odo.Q, &odo.tv, &odo.rv, &dummy, &dummy, dummyN, &odo.ts);
			odometry[index++] = odo;
			// printf("ODOM: %f %f %f %f %f\n" , odo.x, odo.y, odo.Q, dummy, odo.ts);
		}
		
		if (index >= max_num_readings)
			break;
	}

	fclose(f);

	return index;
}

int writeOdometryToFile(vector<odometry_t> odometry, const char * filename)
{
	FILE *f = fopen(filename, "w");
	if (f == NULL)
		return -1;

	fprintf(f, "X Y Theta Ts\n"); // print header - Matlab format
	for (vector<odometry_t>::iterator it = odometry.begin(); it != odometry.end(); ++it)
		fprintf(f, "%f %f %f %f\n", it->x, it->y, it->Q, it->ts);
	fclose(f);

	return 0;
}

int readLidarFromFile(lidar_t* lidar, int max_num_readings)
{
	char line[64 * 1024];
	lidar_t raw;
	int dummyI;
	float dummy;
	char dummyN[10];
	int index = 0;

	FILE *f = fopen(logFile, "r");
	if (f == NULL)
		return -1;

	while (!feof(f))
	{
		int bytes, offset = 0;
		fgets(line, 64 * 1024, f);
		if (strstr(line, "RAWLASER1") == line) // Find 'RAWLASER1' at begining
		{
			sscanf(line, "RAWLASER1 %d %f %f %f %f %f %d %d %n",
				&raw.laser_type, &raw.start_angle, &raw.field_of_view, &raw.angular_resolution,
				&raw.maximum_range, &raw.accuracy, &raw.remission_mode, &raw.num_readings, &bytes);
			offset = bytes;
			for (int i = 0; i < raw.num_readings; ++i)
			{
				sscanf(line + offset, "%f %n", &raw.data[i], &bytes);
				offset += bytes;
			}
			sscanf(line + offset, "%d %f %s %f", &dummyI, &dummy, dummyN, &raw.ts);
			raw.has_odometry = false;
			lidar[index ++] = raw;
		}

		if (strstr(line, "ROBOTLASER1") == line) // Find 'RAWLASER1' at begining
		{
			sscanf(line, "ROBOTLASER1 %d %f %f %f %f %f %d %d %n",
				&raw.laser_type, &raw.start_angle, &raw.field_of_view, &raw.angular_resolution,
				&raw.maximum_range, &raw.accuracy, &raw.remission_mode, &raw.num_readings, &bytes);
			offset = bytes;
			for (int i = 0; i < raw.num_readings; ++i)
			{
				sscanf(line + offset, "%f %n", &raw.data[i], &bytes);
				offset += bytes;
			}
			sscanf(line + offset, "%d %f %f %f %f %f %f %f %f %f %f %f %f %s %f", &dummyI,
				&dummy, &dummy, &dummy, &raw.odom.x, &raw.odom.y, &raw.odom.Q, &raw.odom.tv, &raw.odom.rv, &dummy,
				&dummy, &dummy, &dummy, dummyN, &raw.ts);
			raw.has_odometry = true;
			raw.odom.ts = raw.ts;
			lidar[index++] = raw;
		}

		if (strstr(line, "FLASER") == line) // Find 'RAWLASER1' at begining
		{
			sscanf(line, "FLASER %d %n",
				 &raw.num_readings, &bytes);
			offset = bytes;
			for (int i = 0; i < raw.num_readings; ++i)
			{
				sscanf(line + offset, "%f %n", &raw.data[i], &bytes);
				offset += bytes;
			}
			sscanf(line + offset, "%f %f %f %f %f %f %f %s %f", &dummy, &dummy,
				&dummy, &dummy, &dummy, &dummy, &dummy, dummyN, &raw.ts);
			lidar[index++] = raw;
		}
		raw.start_angle = -1.570796;
		raw.angular_resolution = 0.008727;

		if (index >= max_num_readings)
			break;
	}

	fclose(f);

	return index;
}

void dumpDataPointsToFile(point_t *data_points, int num_points, const char *filename)
{
	FILE *f = fopen(filename, "a");
	assert(f != NULL);

	for (int i = 0; i < num_points; ++i)
		fprintf(f, "(%f,%f) ", data_points[i].x, data_points[i].y);
	fprintf(f, "\n");

	fclose(f);
}

void dumpSegmentsToFile(segment_t *segments, int num_features, const char *filename, int id)
{
	FILE *f = fopen(filename, "a");
	assert(f != NULL);

	for (int i = 0; i < num_features; ++i)
		fprintf(f, "%d %f %f %f %f %f %f %f %f %f\n",id, segments[i].slope,
			segments[i].middle.x, segments[i].middle.y , segments[i].edges[0].x, segments[i].edges[0].y,
			segments[i].edges[1].x, segments[i].edges[1].y, segments[i].m, segments[i].n);
	fprintf(f, "\n");

	fclose(f);
}

void dumpStateToFile(robot_state_t *robot, const char *filename, int id)
{
	FILE *f = fopen(filename, "a");
	assert(f != NULL);

	fprintf(f, "%d %f %f %f\n", id, robot->pos.x, robot->pos.y, robot->Q); 

	fclose(f);
}

void writeLidarToFile(vector<lidar_t> lidar)
{
	FILE *f = fopen("lidar.txt", "w");

	for (vector<lidar_t>::iterator itl = lidar.begin(); itl != lidar.end(); itl++)
	{
		for (int i = 0; i < itl->num_readings-1; ++i)
			fprintf(f, "%f,", itl->data[i]);
		fprintf(f, "%f\n", itl->data[itl->num_readings-1]);
	}
}

void clearFile(const char *filename)
{
	FILE *f = fopen(filename, "w");
	if (f) fclose(f);
}

void dump_ocupancy_map(uint8_t *grid, int grid_size, const char *filename)
{
	FILE *f = fopen(filename, "a");
	int size = grid_size * 2 + 1;
	for (int i = 0; i < size; ++i)
	{
		for (int j = 0; j < size; ++j)
			if (grid[i*size + j] >= 64)
				fprintf(f, "(%d,%d) ", i, j);
	}
	fprintf(f, "\n");
	fclose(f);
}