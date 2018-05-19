#include "Detect.h"
#include "Geometry.h"
#include <iostream>
#include <cmath>
#include <stdlib.h> 
#include "string.h"
#include "assert.h"

using namespace std;

#define PI 3.14159265359

static point_t temp_data[MAX_LIDAR_SAMPLES];
static bool temp_mask[MAX_LIDAR_SAMPLES];

int FeatureDetector::extractSegmentsRansac(segment_t * segments, point_t * data, int num_points)
{
	int target_lines = this->num_lines_target;
	int num_segments = 0;

	// copy data locally - do not tamper with original data
	memcpy(temp_data, data, sizeof(point_t) * num_points);
	point_t *pdata = temp_data;

	for (int i = 0; i < target_lines && num_points > 2; ++i)
	{
		ransac_result_t result;
		int sc = ransac(data, num_points, &result);
		if (sc == 0){
			float m = result.segment.m;
			float n = result.segment.n;

			result.segment.middle.x = (result.segment.edges[0].x + result.segment.edges[1].x) / 2;
			result.segment.middle.y = (result.segment.edges[0].y + result.segment.edges[1].y) / 2;;		
			result.segment.slope = atan(m);

			segments[num_segments++] = result.segment;

			// remove marked points
			int new_index = 0;
			for (int old_index = 0; old_index < num_points; ++old_index)
				if (!result.mask[old_index])
					temp_data[new_index++] = temp_data[old_index];

			assert(new_index == num_points - result.num_points);

			// prepare input for new iteration
			num_points = new_index;

			// skip feature in case it is too close to a previous one
			for (int j = 0; j < num_segments - 1; ++j)
				if (distance(result.segment.middle, segments[j].middle) < 1 )
				{
					num_segments--;
					break;
				}
		}
		else {
			// assert(0);
		}

	}

	return num_segments;
}

int FeatureDetector::ransac(point_t *data, int num_points,  ransac_result_t *result)
{
	int sc = -1;
	int best_num_inliers = 0;

	memset(result, 0, sizeof(result));
	

	for (int i = 0; i < num_iter; ++i)
	{
		// reset mask
		memset(temp_mask, 0, sizeof(temp_mask));
		point_t minp, maxp;
		minp.x = 1000000000.0;
		maxp.x = -1000000000.0;

		// randomly select 2 points
		int idx0 = rand() % num_points;
		int idx1;
		do {
			idx1 = idx0 + ((rand() % 3) - 1)*(rand() % 30);
		} while (idx1 < 0 || idx1 >= num_points || idx1 == idx0);
		if (idx0 == idx1)
			continue;

		point_t p0 = data[idx0];
		point_t p1 = data[idx1];

		// kLine = sample(:, 2) - sample(:, 1); % two points relative distance
		float k_line_x = p1.x - p0.x;
		float k_line_y = p1.y - p0.y;
		// kLineNorm = kLine/norm(kLine);
		float norm = sqrt(k_line_x * k_line_x + k_line_y * k_line_y);
		k_line_x /= norm;
		k_line_y /= norm;
		//  normVector = [-kLineNorm(2),kLineNorm(1)];%Ax+By+C=0 A=-kLineNorm(2),B=kLineNorm(1)
		float norm_vec_x = -k_line_y;
		float norm_vec_y = k_line_x;

		// compute the distances between all points with the fitiing line
		// distance = normVector*(data - repmat(sample(:, 1), 1, number));
		int num_inliers = 0;
		point_t prev;
		for (int i = 0; i < num_points; ++i)
		{
			point_t curr = data[i];
			float dx = curr.x - p0.x;
			float dy = curr.y - p0.y;

			float dist = abs(norm_vec_x * dx + norm_vec_y * dy);
			if (dist < this->thresh_dist)
			{
				if (num_inliers >= 1 && distance(curr, prev) > 1) 
				 	break;

				num_inliers++;
				temp_mask[i] = 1;

				if (curr.x < minp.x) minp = curr;
				if (curr.x > maxp.x) maxp = curr;

				prev = curr;
			}
		}

		if (num_inliers > this->inlier_thresh && num_inliers > best_num_inliers)
		{
			best_num_inliers = num_inliers;
			// a = (yb - ya) / (xb - xa)
			result->segment.m = k_line_y / k_line_x;
			result->segment.n = p0.y - p0.x * result->segment.m;
			leastSquare(data, num_points, temp_mask, &result->segment.m, &result->segment.n);
			result->segment.edges[0] = minp;
			result->segment.edges[1] = maxp;
			result->num_points = best_num_inliers;
			result->segment.var = computeVariance(data, num_points, result->segment.m, result->segment.n);
			memcpy(result->mask, temp_mask, num_points);

			sc = 0; // valid results
			break;
		}
	}


	return sc;
}

void FeatureDetector::leastSquare(point_t *data, int num_points, bool *mask, float *a, float *b)
{
	float sumx = 0, sumy = 0;
	float sumx2 = 0, sumy2 = 0;
	float sumxy = 0;
	int count = 0;

	for (int i = 0; i < num_points; ++i)
	{
		if (mask[i] == false)
			continue;

		sumx += data[i].x;
		sumy += data[i].y;
		sumx2 += data[i].x * data[i].x;
		sumy2 += data[i].y * data[i].y;
		sumxy += data[i].x * data[i].y;

		count++;
	}

	*b = (sumy*sumx2 - sumx*sumxy) / (count*sumx2 - sumx*sumx);
	*a = (count*sumxy - sumx*sumy) / (count*sumx2 - sumx*sumx);
}

float FeatureDetector::computeVariance(point_t *data, int num_points, float m, float n)
{
	float var = 0;
	float a = m;
	float b = -1;
	float c = n;
	float num = sqrt(a*a + b*b);

	for (int i = 0; i < num_points; ++i)
	{
		float dist = abs(a*data[i].x + b*data[i].y + c) / num;
		var += dist*dist;
	}
	var /= num_points;

	return var;
}

int FeatureDetector::filterSegments(segment_t *segment, int num_segments, float slope_th, float dist_th)
{
	bool *mark = new bool[num_segments];
	
	for (int s0 = 0; s0 < num_segments; ++s0)
		mark[s0] = false;

	for (int s0 = 0; s0 < num_segments; ++s0)
	{
		for (int s1 = s0 + 1; s1 < num_segments && mark[s0] == false; ++s1)
		{
			if (mark[s1] == true) continue;
			float a0 = segment[s0].slope;
			float a1 = segment[s1].slope;
			if (a0 < 0) a0 += PI;
			if (a1 < 0) a1 += PI;
			if (abs(a0 - a1) < slope_th)
			{
				if (segment[s0].var < segment[s1].var)
					mark[s1] = true;
				else
					mark[s0] = true;
			}
		}
	}

	int count = 0;
	for (int s = 0; s < num_segments; ++s)
		if (!mark[s])
			segment[count++] = segment[s];

	delete[] mark;

	return count;
}

int FeatureDetector::intersectSegments(segment_t *segments, int num_segments, point_t *landmarks)
{
	int num_landmarks = 0;

	for (int s0 = 0; s0 < num_segments; ++s0)
	{
		for (int s1 = s0 + 1; s1 < num_segments; ++s1) {
			if (abs(segments[s0].m - segments[s1].m) < 0.001)
				continue;
			point_t inter;
			inter.x = (segments[s1].n - segments[s0].n) / (segments[s0].m - segments[s1].m);
			inter.y = segments[s0].m * inter.x + segments[s0].n;

			// Sanity check
			// TODO: threshold should be dependent on the laser sensor
			if (abs(inter.x) > 10.0 || abs(inter.y) > 10.0)
				continue;

			landmarks[num_landmarks++] = inter;
		}
	}

	return num_landmarks;
}