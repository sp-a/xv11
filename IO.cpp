#include "IO.h"
#include "assert.h"
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>

#define PI 3.14159265359

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

		tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
		tty.c_oflag     &=  ~OPOST;              // make raw

		/* Flush Port, then applies attributes */
		tcflush( fd, TCIFLUSH );

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}


int readOdometryFromFile(char *logFile, odometry_t *odometry, int max_num_readings)
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

int readLidarFromFile(char *logFile, lidar_t* lidar, int max_num_readings)
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
		// if (strstr(line, "RAWLASER1") == line) // Find 'RAWLASER1' at begining
		// {
		// 	sscanf(line, "RAWLASER1 %d %f %f %f %f %f %d %d %n",
		// 		&raw.laser_type, &raw.start_angle, &raw.field_of_view, &raw.angular_resolution,
		// 		&raw.maximum_range, &raw.accuracy, &raw.remission_mode, &raw.num_readings, &bytes);
		// 	offset = bytes;
		// 	for (int i = 0; i < raw.num_readings; ++i)
		// 	{
		// 		sscanf(line + offset, "%f %n", &raw.data[i], &bytes);
		// 		offset += bytes;
		// 	}
		// 	sscanf(line + offset, "%d %f %s %f", &dummyI, &dummy, dummyN, &raw.ts);
		// 	raw.has_odometry = false;
		// 	lidar[index ++] = raw;
		// }

		// if (strstr(line, "ROBOTLASER1") == line) // Find 'RAWLASER1' at begining
		// {
		// 	sscanf(line, "ROBOTLASER1 %d %f %f %f %f %f %d %d %n",
		// 		&raw.laser_type, &raw.start_angle, &raw.field_of_view, &raw.angular_resolution,
		// 		&raw.maximum_range, &raw.accuracy, &raw.remission_mode, &raw.num_readings, &bytes);
		// 	offset = bytes;
		// 	for (int i = 0; i < raw.num_readings; ++i)
		// 	{
		// 		sscanf(line + offset, "%f %n", &raw.data[i], &bytes);
		// 		offset += bytes;
		// 	}
		// 	sscanf(line + offset, "%d %f %f %f %f %f %f %f %f %f %f %f %f %s %f", &dummyI,
		// 		&dummy, &dummy, &dummy, &raw.odom.x, &raw.odom.y, &raw.odom.Q, &raw.odom.tv, &raw.odom.rv, &dummy,
		// 		&dummy, &dummy, &dummy, dummyN, &raw.ts);
		// 	raw.has_odometry = true;
		// 	raw.odom.ts = raw.ts;
		// 	lidar[index++] = raw;
		// }

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

#include <string.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace cv;
float dists[360];
uint8_t smap[500][500];
uint8_t imu[16];
uint8_t packet[24];
uint64_t imu_ts = 0;
uint64_t lidar_ts = 0;
uint64_t ts0 = 0;

void dumpScanToFile(const char *filename, uint64_t ts)
{
	printf("dumpScanToFile %s\n", filename);
	FILE *f = fopen(filename, "a");
	assert(f != NULL);
	for(int i = 0 ; i < 360; ++i )
	{
		fprintf(f, "%f ", dists[i]);
	}
	fprintf(f, "%lu\n", ts);
	fclose(f);
}

void dumpImuToFile(const char * filename, uint64_t ts)
{
	printf("dumpImuToFile %s\n", filename);
	FILE *f = fopen(filename, "a");
	assert(f != NULL);
	for(int i = 0; i < 12; ++i )
		fprintf(f, "%u ", imu[i]);
	fprintf(f, "%lu\n", imu_ts);
	fclose(f);
}

static void readSensor(int fd, uint8_t *buf, int  size, int filter)
{
	int bytes = 0;
	uint8_t *ptr = buf;
	do
	{
		int rd =  read (fd, ptr + bytes, size-bytes);
		if(filter)
		{
			int count = 0;
			for(int i = 0; i < rd ; ++i )
			 	if(*(ptr + bytes + i) != 0xFF)
			 		*(ptr + bytes + count ++) = *(ptr + bytes + i);
			rd = count;
		}
		ptr += rd;
		bytes += rd;
	}while(bytes < size);
}


void showmap()
{
	int count = 0;
	memset(smap, 0 , sizeof(smap));
	for(int i = 0; i < 360; ++i )
	{
		if(dists[i] == 0 || dists[i] > 10.0) continue;
		float angle  = (float)i * DEGREES_TO_RADIAN;
		float x = cos(angle) * dists[i];
		float y = sin(angle) * dists[i];

		int dx = (int)roundf(x / 0.05) + 250;
		int dy = (int)roundf(y / 0.05) + 250;
		smap[dx][dy] = 255;
		count ++;
		// printf("dx %d dy %d\n", dx , dy);	
	}

	   	Mat gmap(500, 500, CV_8UC1, smap);
		namedWindow( "Global map", WINDOW_AUTOSIZE );// Create a window for display.
   		Mat invgmap, szgmap;
   		bitwise_not ( gmap, invgmap ); //(, invgrid, 0, 255, CV_THRESH_BINARY_INV);
   		resize(invgmap, szgmap, cvSize(1280, 720));
   		imshow( "Global map" , szgmap );
   		printf("%d points detected!\n", count);

   		waitKey(1); 
}

int checksum(uint8_t *data)
{
	int temp;
	int chk32 = 0;

	for(int i = 0; i < 10; i++)
	{
		temp = data[2*i] + ((int)data[2*i+1] << 8);
		chk32 = (chk32 << 1) + temp;
	}

	printf("chk32: %x\n", chk32);

	int checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ); //  wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF; // truncate to 15 bits

    return checksum;
}

uint64_t GetTimeStamp() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

int openSerialPort(const char *devname)
{
	int fd = open (devname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
	        printf ("error %d opening %s: %s", errno, devname, strerror (errno));
	        return fd;
	}
	printf("file descriptor: %d\n", fd);

	set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 1);                // set no blocking

	ts0 = GetTimeStamp();

	return fd;
}

int readFromSerialPort(int fd)
{	
	int speed, index;
	int previndex = 0, init_level = 0;
	while(1)
	{
		
		if (init_level == 0 )
		{
               readSensor(fd, &packet[0], 1, 1);

               if(packet[0] == 0xFA ) 
               {
               		lidar_ts = GetTimeStamp();
                    init_level = 1;
               }
               else
               {
               	   if(packet[0] == 0xfb)
               	   {
               	   		uint8_t check;
               	   		readSensor(fd, &check, 1, 0);
               	   		if(check == 0xcd){
               	   			imu_ts  =GetTimeStamp() - ts0;
	               	   	    // printf("imu read!\n");

	               	   	    // force split ???
	               	   		readSensor(fd, &imu[0], 2, 0);
	               	   		readSensor(fd, &imu[2], 2, 0);
	               	   		readSensor(fd, &imu[4], 2, 0);
	               	   		readSensor(fd, &imu[6], 2, 0);
	               	   		readSensor(fd, &imu[8], 2, 0);
	               	   		readSensor(fd, &imu[10], 2, 0);
	               	   		dumpImuToFile("imu.txt", imu_ts - ts0);
	               	   		 return 1;
               	   	}
               	   }
                   init_level = 0;
               }
        }
        else if (init_level == 1)
        {
        		readSensor(fd, &packet[1], 1, 1);

                // position index 
                if(packet[1] >= 0xA0 && packet[1] <= 0xF9 ) 
                {
                    index = packet[1] - 0xA0;
                    init_level = 2;
                }
                else
                {
                	printf("bad index: 0x%x\n", packet[1]);
                    init_level = 0;
                }
        } else if (init_level == 2)
        {
                // speed
        		readSensor(fd, &packet[2], 2,  1);
        		{
                	speed = ((int)packet[3] << 8) | packet[2];
                    //printf("speed: %f RPM %x\n", (float)speed / 64, speed);
        		}
                
                
                // data
                {
                	for(int i = 0; i < 4 ; ++i )
                	{
                		readSensor(fd, &packet[4 + 4 * i], 2, 1);
                		int data = (packet[4 + 4 * i] | ((int)(packet[4 + 4*i + 1] & 0xff )<< 8));

                		if(data & 0x4000)
                		{
                			printf("warning! \n");
                			continue;
                		}
                		if(data & 0x8000)
                		{
                			printf("error code: 0x%x!\n", data & 0xFF);
                			continue;
                		}

                		// printf("data: %x\n", data);
                		dists[index * 4 + i] =  (float)data/1000;

                		// intensity
                		readSensor(fd, &packet[4 + 4 * i + 2], 2, 1);
                	}
				}

                // checksum
                readSensor(fd, &packet[20], 2, 1);
                int check = packet[20] |( (int)packet[21] << 8);

               	// if(check == checksum(packet))
               	// 	printf("checksum ok!\n");
               	// else
               	// 	printf("checksum error 0x%x vs 0x%x, 0x%x, 0x%x!\n", checksum(packet), check, packet[21], packet[20]);

                init_level = 0;

                if(index < previndex)
                {
                	previndex = index;
                	dumpScanToFile("lidar.txt", lidar_ts - ts0);
                	return 0;
                }
                previndex = index;
        }

	// 	if(n <= 0)
	// 		printf("read error!\n");
	}
	return 0;
}