#include "Grid.h"



float cos_lut[360], sin_lut[360];

void init_trig_lut()
{
	for (int angle = 0; angle < 360; ++angle)
	{
		cos_lut[angle] = cos(angle * DEGREES_TO_RADIAN);
		sin_lut[angle] = sin(angle * DEGREES_TO_RADIAN);
	}
}

// grid size in 
void get_local_ocupancy_grid(point_t *data, int num_data, uint8_t grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE],
							 float step, int grid_size)
{
	int size = grid_size * 2 + 1;
	int max_index = size * size;
	for (int i = 0; i < size; ++i)
		for(int j = 0 ; j < size; ++j)
			grid[i][j] = 0; // unknown

	for (int i = 0; i < num_data; ++i)
	{
		data[i].x = roundf(data[i].x / step);
		data[i].y = roundf(data[i].y / step);
		int x = (int)data[i].x + grid_size;
		int y = (int)data[i].y + grid_size;

		if (x > size || y > size)
			continue;
		grid[x][y] = 1;
	}
}

void extract_local_grid(uint8_t g_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE], int g_range,
	uint8_t l_grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE], int l_range,
						float angle, int rx, int ry)
{
	int degrees = (int)(angle * RADIAN_TO_DEGREE);
	while (degrees > 360) degrees -= 360;
	while (degrees < 0) degrees += 360;
	int g_size = g_range * 2 + 1;

	rx += g_range;
	ry += g_range;

	for (int x = -l_range; x < l_range; x++)
	{
		for (int y = -l_range; y < l_range; ++y)
		{
			int tx = roundf(cos_lut[degrees] * x - sin_lut[degrees] * y);
			int ty = roundf(sin_lut[degrees] * x + cos_lut[degrees] * y);

			if (rx + tx < 0 || rx + tx >= g_size || ry + ty < 0 || ty + ty >= g_size)
				l_grid[x + l_range][y + l_range] = 0;
			else
				l_grid[x + l_range][y + l_range] = g_grid[rx + tx][ry + ty];
		}
	}
}

int extract_local_points(uint8_t g_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE], int g_range,
	point_t *points, int l_range, float angle, int rx, int ry)
{
	int degrees = (int)(angle * RADIAN_TO_DEGREE);
	while (degrees > 360) degrees -= 360;
	while (degrees < 0) degrees += 360;
	int g_size = g_range * 2 + 1;
	int num_points = 0;

	rx += g_range;
	ry += g_range;

	for (int x = -l_range; x < l_range; x++)
	{
		for (int y = -l_range; y < l_range; ++y)
		{
			int tx = roundf(cos_lut[degrees] * x - sin_lut[degrees] * y);
			int ty = roundf(sin_lut[degrees] * x + cos_lut[degrees] * y);

			if (rx + tx < 0 || rx + tx >= g_size || ry + ty < 0 || ty + ty >= g_size)
				;
			else
			{
				//l_grid[x + l_range][y + l_range] = g_grid[rx + tx][ry + ty];
				points[num_points].x = tx;
				points[num_points].y = ty;
				num_points++;
			}
		}
	}

	return num_points;
}

void update_map(uint8_t g_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE], int g_range,
	uint8_t l_grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE], int l_range, float angle, int rx, int ry)
{
	int degrees = (int)(angle * RADIAN_TO_DEGREE);
	while (degrees > 360) degrees -= 360;
	while (degrees < 0) degrees += 360;
	int g_size = g_range * 2 + 1;

	rx += g_range;
	ry += g_range;

	for (int x = -l_range; x < l_range; ++x)
	{	
		for(int y = -l_range; y < l_range ; ++y)
			if (l_grid[x + l_range][y + l_range] == 1)
			{
				int tx = roundf(cos_lut[degrees] * x - sin_lut[degrees] * y);
				int ty = roundf(sin_lut[degrees] * x + cos_lut[degrees] * y);
				if (rx + tx < 0 || rx + tx >= g_size || ry + ty < 0 || ty + ty >= g_size);
				else
				{	
					if (g_grid[rx + tx][ry + ty] < 255-16)
						g_grid[rx + tx][ry + ty] += 16;
				}
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
			if (l_grid[x][y] >= 1)
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



point_t temp_points[LOCAL_GRID_SIZE*LOCAL_GRID_SIZE];
void scan_to_map(uint8_t l_grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE], int l_range,
	point_t *points, int num_points, int *angle, int *rx, int *ry, int *cost, int *matches)
{
	int best_cost = 0x1FFFFFFF;
	int best_dx, best_dy, best_dangle, best_matches;
	int init_cost = 0;

	for (int i = 0; i < LOCAL_GRID_SIZE; ++i)
		for (int j = 0; j < LOCAL_GRID_SIZE; ++j)
			init_cost += l_grid[i][j];

	for (int dx = 0; dx <= 3 ; )
	{
		for (int dy = 0; dy <= 3; )
		{
			for (int dangle = 0; dangle <= 20; )
			{
				//memset(temp_grid, 0, sizeof(temp_grid));
				//transform_grid(l_grid, temp_grid, LOCAL_GRID_MAX_RANGE, dangle, dx, dy);
				transform_points(points, num_points, temp_points, dangle, dx, dy);
				int matches;
				int cost = compute_grid_diff(l_grid, LOCAL_GRID_MAX_RANGE, temp_points, num_points, init_cost, &matches);
				//printf("transform: %d %d %d cost %d\n", dangle, dx, dy, cost);

				if (cost < best_cost)
				{
					best_cost = cost;
					best_dx = dx;
					best_dy = dy;
					best_dangle = dangle;
					best_matches = matches;
					//printf("transform: %d %d %d cost %d %d %d\n", dangle, dx, dy, init_cost, cost, matches);
				}

				dangle = -dangle;
				if (dangle >= 0) dangle++;
			}

			dy = -dy;
			if (dy >= 0) dy++;
		}
		dx = -dx;
		if (dx >= 0) dx++;
	}


	*angle = best_dangle;
	*rx = best_dx;
	*ry = best_dy;
	*cost = best_cost;
	*matches = best_matches;
}

void post_process_grid_erode(uint8_t grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE], uint8_t pp_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE],  int size)
{
	for (int i = 1; i < size - 1; ++i)
		for (int j = 1; j < size - 1; ++j)
			pp_grid[i][j] = grid[i - 1][j] * 
							grid[i][j - 1] * grid[i][j] * grid[i][j + 1] *
							grid[i + 1][j];
}