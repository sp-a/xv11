#pragma once
#include "customTypes.h"

void init_trig_lut();
void get_local_ocupancy_grid(point_t *data, int num_data, uint8_t grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE],
	float step, int grid_size);
void extract_local_grid(uint8_t g_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE], int g_range,
	uint8_t l_grid[LOCAL_GRID_PADDED_SIZE][LOCAL_GRID_PADDED_SIZE], int l_range,
	float angle, int rx, int ry);
int extract_local_points(uint8_t g_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE], int g_range,
	point_t *points, int l_range, float angle, int rx, int ry);
void update_map(uint8_t g_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE], int g_range,
	uint8_t l_grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE], int l_range,
	float angle, int rx, int ry);
void scan_to_map(uint8_t l_grid[LOCAL_GRID_SIZE][LOCAL_GRID_SIZE], int l_range,
	uint8_t g_grid[LOCAL_GRID_PADDED_SIZE][LOCAL_GRID_PADDED_SIZE], int g_range,
	 int *angle, int *rx, int *ry, int *cost,  int *matches);
void post_process_grid_erode(uint8_t grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE],
	uint8_t pp_grid[GLOBAL_GRID_SIZE][GLOBAL_GRID_SIZE], int size);
