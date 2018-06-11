#include "Grid.h"
#include "Geometry.h"
#include "icpPointToPlane.h"
#include "icpPointToPoint.h"

int freq[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE];
int occ[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE];

int last_seen[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE];

float cos_lut[360], sin_lut[360];

void init_trig_lut()
{
	for (int angle = 0; angle < 360; ++angle)
	{
		cos_lut[angle] = cos(angle * DEGREES_TO_RADIAN);
		sin_lut[angle] = sin(angle * DEGREES_TO_RADIAN);
	}

	memset(freq, 0, sizeof(freq));
	memset(occ, 0, sizeof(occ));
	memset(last_seen, 0, sizeof(last_seen));
}

// grid size in 
void get_local_ocupancy_grid(point_t *data, int num_data, uint8_t grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE],
							 float step, int grid_size)
{
	// Initialize local grid with unknown state
	int free = 10;
	int size = grid_size * 2 + 1;
	int max_index = size * size;
	for (int i = 0; i < size; ++i)
		for(int j = 0 ; j < size; ++j)
			grid[i][j] = 128; // unknown

	// Mark all current occupied cells
	for (int i = 0; i < num_data; ++i)
	{
		int x = (int)roundf(data[i].x / step) + grid_size;
		int y = (int)roundf(data[i].y / step) + grid_size;

		if (x < 0 || y < 0 || x > size || y > size)
			continue;

		grid[x][y] = 255; // occupied
	}

	// Filter noise
	int k_size = 5;
	for(int i = k_size ; i < size - k_size ; ++i )
		for(int j = k_size; j < size - k_size; ++j )
			if(grid[i][j] == 255)
			{
				// check this point for noise
				int count = 0;
				for(int dx = -k_size ; dx < k_size ; ++dx )
					for(int dy = -k_size ; dy < k_size ; ++dy )
						if(grid[i + dx][j + dy] == 255) 
							count ++;

				if(count < 3)
					grid[i][j] = 128;
				//printf("count: %d\n",count );
			}

	// Mark free cells
	for (int i = 0; i < num_data; ++i)
	{
		int x = (int)roundf(data[i].x / step) + grid_size;
		int y = (int)roundf(data[i].y / step) + grid_size;

		if (x < 0 || y < 0 || x > size || y > size || grid[x][y] != 255)
			continue;

		// Mark all cell in between as free cells
		// How many points on the line?
		int num_p = (int)sqrt( data[i].x * data[i].x + data[i].y * data[i].y) / step;
		//printf("num_p: %d %f %f\n" , num_p, data[i].x, data[i].y);
		// A(0, 0), B(data[i].x, data[i].y)
		for(int k1 = 1; k1 < num_p - 1 ; ++k1 )
		{
			int x = (int)roundf(k1 * data[i].x / (num_p * step)) + grid_size;
			int y = (int)roundf(k1 * data[i].y / (num_p * step)) + grid_size;
				grid[x][y] = free; // free
		}
	}

	// Dilate free cells
	for(int i = 1 ; i < size - 1 ; ++i )
		for(int j = 1; j < size - 1; ++j )
		{
			if(grid[i][j] != 128) continue; // only unknown cells
				if(grid[i-1][j] == free && grid[i+1][j] == free && 
				   grid[i-1][j] == free && grid[i+1][j] == free)
						grid[i][j] = free; // mark as free
		}
}

void extract_local_grid(uint8_t g_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE], int g_range,
	uint8_t l_grid[LOCAL_GRID_PADDED_SIZE][LOCAL_GRID_PADDED_SIZE], int l_range,
						float angle, int rx, int ry)
{
	int g_size = g_range * 2 + 1;
	static int iter = 0;
	iter ++;
	rx += g_range;
	ry += g_range;

	for (int x = -l_range; x < l_range; x++)
	{
		for (int y = -l_range; y < l_range; ++y)
		{
			int tx = roundf(cos(angle) * x - sin(angle) * y);
			int ty = roundf(sin(angle) * x + cos(angle) * y);

			if (rx + tx < 0 || rx + tx >= g_size || ry + ty < 0 || ty + ty >= g_size)
				l_grid[x + l_range][y + l_range] = 128;
			else
			{
				if(g_grid[rx + tx][ry + ty] > 240 )
					l_grid[x + l_range][y + l_range] = g_grid[rx + tx][ry + ty];
				else
					l_grid[x + l_range][y + l_range] = 128;
			}
		}
	}
}

int extract_local_points(uint8_t g_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE], int g_range,
	point_t *points, int l_range, float angle, int rx, int ry)
{
	static int iter = 0;
	int g_size = g_range * 2 + 1;
	int num_points = 0;
	iter ++;

	rx += g_range;
	ry += g_range;

	for (int x = -l_range; x < l_range; x++)
	{
		for (int y = -l_range; y < l_range; ++y)
		{
			int tx = roundf(cos(angle) * x - sin(angle) * y);
			int ty = roundf(sin(angle) * x + cos(angle) * y);

			if (rx + tx < 0 || rx + tx >= g_size || ry + ty < 0 || ty + ty >= g_size)
				;
			else
			{
				if(g_grid[rx + tx][ry + ty] > 240  )
				{
					//l_grid[x + l_range][y + l_range] = g_grid[rx + tx][ry + ty];
					points[num_points].x = tx;
					points[num_points].y = ty;
					num_points++;

					// printf("iter %d last_seen %d\n", iter , last_seen[rx + tx][ry + ty] );
				}
			}
		}
	}

	return num_points;
}

void update_map(uint8_t g_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE], int g_range,
	uint8_t l_grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE], int l_range, float angle, int rx, int ry)
{
	int g_size = g_range * 2 + 1;
	static int iter = 0;
	iter ++;
	rx += g_range;
	ry += g_range;

	for (int x = -l_range; x < l_range; ++x)
	{	
		for(int y = -l_range; y < l_range ; ++y)
		{
			int tx = roundf(cos(angle) * x - sin(angle) * y);
			int ty = roundf(sin(angle) * x + cos(angle) * y);

			int val = l_grid[x + l_range][y + l_range];
			if(val == 128) continue;
			// if(val == 0) continue;

			if(0) // Bayes filter
			{
				float Px = (float)val / 256;
				float trx = Px / (1 - Px);
				float Px_1 = (float)g_grid[rx + tx][ry + ty] / 256;
				float trx_1 = Px_1 / (1 - Px_1);
				// printf ("px %f px_1 %f\n",  Px, Px_1);

				float pp = trx * trx_1 / (1 + trx *  trx_1);
				g_grid[rx + tx][ry + ty] = (int)(pp * 255);
				// printf("pp: %f\n",  pp);
				continue;
			}


			
			// freq[rx + tx][ry + ty] ++;
					
			if(val == 255)
			{
				occ[rx + tx][ry + ty] ++;	
				freq[rx + tx][ry + ty] ++;
				last_seen[rx + tx][ry + ty] = iter;
			}

			// if(val == 0 && g_grid[rx + tx][ry + ty] == 255 && 
			// 	last_seen[rx + tx][ry + ty] - iter > 100)
			// 		freq[rx + tx][ry + ty] ++;
			
			float prob = (float) occ[rx + tx][ry + ty] / freq[rx + tx][ry + ty];
			g_grid[rx + tx][ry + ty] = (int)roundf(prob * 255);		
		}
	}
}

void transform_grid(uint8_t grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE], uint8_t t_grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE],
	int l_range, int dangle, int dx, int dy)
{
	if (dangle < 0) dangle += 360;

	for (int x = -l_range; x < l_range; ++x)
	{
		for (int y = -l_range; y < l_range; ++y)
			if (grid[x + l_range][y + l_range] == 1)
			{
				int tx = roundf(cos_lut[dangle] * x - sin_lut[dangle] * y + dx);
				int ty = roundf(sin_lut[dangle] * x + cos_lut[dangle] * y + dy);
				if (tx >= -l_range && tx < l_range && ty >= -l_range && ty < l_range)
					t_grid[tx + l_range][ty + l_range] = 1;
			}
	}
}

void transform_points(point_t *points, int num_points, point_t *temp_points, int dangle, int dx, int dy)
{
	if (dangle < 0) dangle += 360;

	for(int i = 0; i < num_points; ++i)
	{
		temp_points[i].x = ( cos_lut[dangle] * points[i].x - sin_lut[dangle] * points[i].y + dx);
		temp_points[i].y = ( sin_lut[dangle] * points[i].x + cos_lut[dangle] * points[i].y + dy);	 
	}
}

int compute_grid_diff(uint8_t l_grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE], int l_range, point_t *points, int num_points, int init_cost, int *matches)
{
	int sum = init_cost;
	int l_size = l_range * 2 + 1;
	int count = 0;

	for (int i = 0; i < num_points; ++i)
	{
		int x = roundf(points[i].x) + l_range;
		int y = roundf(points[i].y) + l_range;

		if (x < 0 || x > l_size || y < 0 || y > l_size)
			sum++;
		else
			if (l_grid[x][y] >= 16)
			{
				sum -= l_grid[x][y];
				count++;
			}
			else
				sum+=128;
	}

	*matches = count;
	return sum;
}

float ddist(int x0, int y0, int x1, int y1)
{
	return (float)sqrt((x0-x1)^2 + (y0-y1)^2);
}

void scan_to_map(uint8_t map[LOCAL_GRID_PADDED_SIZE][LOCAL_GRID_PADDED_SIZE], int g_range,
				 point_t *scan, int num_scan,
				 float *angle, float *rx, float *ry, float *residual)
{
	int best_cost = 1000000000;
	float best_dx, best_dy, best_dangle, best_matches;
	
	for (float dx = 0; dx <= 10 ; )
	{
		for (float dy = 0; dy <= 10; )
		{
			for (float dangle = 0; dangle <= 20 * 0.0174532925; )
			{
				int cost = 0;

				for(int ind = 0; ind < num_scan ; ++ind)
				{
					float nx = cos(dangle) * scan[ind].x - sin(dangle) * scan[ind].y + dx;
					float ny = sin(dangle) * scan[ind].x + cos(dangle) * scan[ind].y + dy;

					int dnx = (int)roundf(nx) + g_range;
					int dny = (int)roundf(ny) + g_range;

					cost += 255 - map[dnx][dny];
				}

				if (cost < best_cost)
				{
					best_cost = cost;
					best_dx = dx;
					best_dy = dy;
					best_dangle = dangle;
					// printf("    transform: %f %f %f cost %d\n", dangle, dx, dy, cost);
				}

				dangle = -dangle;
				if (dangle >= 0) dangle += 1 * 0.0174532925;
			}

			dy = -dy;
			if (dy >= 0) dy += 1;
		}
		dx = -dx;
		if (dx >= 0) dx += 1;
	}

	// for (float dx = best_dx; dx <= best_dx + 1; )
	// {
	// 	for (float dy = best_dy; dy <= best_dy + 1; )
	// 	{
	// 		for (float dangle = best_dangle; dangle <= best_dangle + 3 * 0.0174532925; )
	// 		{
	// 			int cost = 0;

	// 			for(int ind = 0; ind < num_scan ; ++ind)
	// 			{
	// 				float nx = cos(dangle) * scan[ind].x - sin(dangle) * scan[ind].y + dx;
	// 				float ny = sin(dangle) * scan[ind].x + cos(dangle) * scan[ind].y + dy;

	// 				int dnx = (int)roundf(nx) + g_range;
	// 				int dny = (int)roundf(ny) + g_range;

	// 				cost += 255 - map[dnx][dny];
	// 			}

	// 			if (cost < best_cost)
	// 			{
	// 				best_cost = cost;
	// 				best_dx = dx;
	// 				best_dy = dy;
	// 				best_dangle = dangle;
	// 				// printf("    transform: %f %f %f cost %d\n", dangle, dx, dy, cost);
	// 			}

	// 			dangle = -dangle;
	// 			if (dangle >= 0) dangle += 0.25 * 0.0174532925;
	// 		}

	// 		dy = -dy;
	// 		if (dy >= 0) dy += 0.25;
	// 	}
	// 	dx = -dx;
	// 	if (dx >= 0) dx += 0.25;
	// }


	*angle = best_dangle;
	*rx = best_dx;
	*ry = best_dy;
	*residual = best_cost;
}

void post_process_grid_erode(uint8_t grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE], uint8_t pp_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE],  int size)
{
	for (int i = 1; i < size - 1; ++i)
		for (int j = 1; j < size - 1; ++j)
			pp_grid[i][j] = grid[i - 1][j] * 
							grid[i][j - 1] * grid[i][j] * grid[i][j + 1] *
							grid[i + 1][j];
}