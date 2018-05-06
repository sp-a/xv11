#pragma once

#include "customTypes.h"

typedef struct {
	segment_t segment;
	bool mask[MAX_LIDAR_SAMPLES];
	int num_points;
}ransac_result_t;

class FeatureDetector
{
public:
	FeatureDetector(int num_lines_target, int inlier_thresh, float thresh_dist, int num_iter)
	{
		this->num_lines_target = num_lines_target;
		this->inlier_thresh = inlier_thresh;
		this->thresh_dist = thresh_dist;
		this->num_iter = num_iter;
	}

	~FeatureDetector()
	{
	}

	int extractSegmentsRansac(segment_t * segments, point_t * points, int num_points);
	int filterSegments(segment_t *segment, int num_segments, float slope_th, float dist_th);
	int intersectSegments(segment_t *segment, int num_segments, point_t *landmarks);

private:
	int num_lines_target;
	int inlier_thresh; // minimum number of points on a given line
	float thresh_dist; // maximum distance between any point and the fited line
	int num_iter;

	int ransac(point_t *points, int num_points, ransac_result_t *result);
	void leastSquare(point_t *points, int num_points, bool *mask, float *m, float *n);
	float computeVariance(point_t *data, int num_points, float m, float n);
	
};
