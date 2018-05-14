#include "IO.h"
#include "assert.h"
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

static const char* logFile = "dataset.log";

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
        tty.c_cc[VTIME] = 50;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

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
        tty.c_cc[VTIME] = 10;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}


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
			 // printf("ODOM:%d %f %f %f %f %f\n" ,index, odo.tv, odo.rv, odo.Q, dummy, odo.ts);
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

void readFromSerialPort(const char *devname)
{
	
	int fd = open (devname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
	        printf ("error %d opening %s: %s", errno, devname, strerror (errno));
	        return;
	}

	set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 1);                // set no blocking

	// uint8_t byte;

	// while(1){
	// 	int n = read (fd, &byte, 1);
	// 	if(byte == 0xfa)
	//    		 printf("\n");
	// 	printf("0x%2x ", byte);
	// }
	// write (fd, "hello!\n", 7);           // send 7 character greeting

	// usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
	                                     // receive 25:  approx 100 uS per char transmit
	uint8_t byte;
	uint8_t array[16];
	int init_level = 0;
	int speed, index;
	int data[4];
	while(1)
	{
		
		if (init_level == 0 )
		{
               int n = read (fd, &byte, 1);
               if( n == 1 && byte == 0xFA ) 
                   init_level = 1;
               else
                   init_level = 0;
        }
        else if (init_level == 1)
        {
        		int n = read (fd, &byte, 1);
                // position index 
                if(n == 1 && byte >= 0xA0 && byte <= 0xF9 ) 
                {
                    index = byte - 0xA0;
                    init_level = 2;
                }
                else
                {
                	printf("bad index!!!!\n");
                    init_level = 0;
                }
        } else if (init_level == 2)
        {
                // speed
        		int n = read (fd, array, 2);
        		if(n == 2)
        		{
                	speed = ((int)array[1] << 8) | array[0];
                	printf("speed: %f RPM\n", (float)speed / 64);
        		}
                else
                {
                	 init_level = 0;
                	 continue;
                } 
                
                n = read (fd, array, 16);
                // data
                if(n == 16)
                {
                	for(int i = 0; i < 4 ; ++i )
                	{
                		data[i] = ((int)(0x3f & array[i*4 + 1])) | array[i*4];
                		if(array[i*4 + 1] & 0x40)
                			printf("warning! ");
                		if(array[i*4 + 1] & 0x80)
                		{
                			printf("error code: 0x%x!\n", array[i*4]);

                		}
                		else
                		{

                			printf("angle: %d  dist %f \n", index * 4 + i, (float)data[i]/1000 );
                		}
                	}
				}
				else
                {
                	 printf("bad data!\n");
                	 init_level = 0;
                	 continue;
                } 

                // checksum
                n = read (fd, array, 2);
                if(n != 2)
                	printf("bad checksum!\n");

                // # for the checksum, we need all the data of the packet...
                // # this could be collected in a more elegent fashion... 
                // all_data = [ 0xFA, index+0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3 

                // # checksum  
                // b_checksum = [ ord(b) for b in ser.read(2) ]
                // incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)

                init_level = 0;
        }

	// 	if(n <= 0)
	// 		printf("read error!\n");
	}
}