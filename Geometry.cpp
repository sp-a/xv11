#include "Geometry.h"

void transform_point(point_t *p, float dx, float dy, float Q)
{
	point_t original = *p;
	p->x = cos(Q) * original.x - sin(Q)*original.y + dx;
	p->y = sin(Q) * original.x + cos(Q)*original.y + dy;
}

void transform_points(point_t *p, int num_points, float x, float y, float q)
{
	for (int i = 0; i < num_points; ++i)
		transform_point(&p[i], x, y, q);
}

float distance(point_t a, point_t b)
{
	return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

float sg_length(segment_t seg)
{
	return sqrt((seg.edges[0].x - seg.edges[1].x) * (seg.edges[0].x - seg.edges[1].x) +
		(seg.edges[0].y - seg.edges[1].y) * (seg.edges[0].y - seg.edges[1].y));
}

void transform_segment(segment_t *sg, float x, float y, float angle)
{
	sg->slope += angle;
	while (sg->slope < 0) sg->slope += PI;
	while (sg->slope > PI) sg->slope -= PI;

	transform_point(&sg->middle, x, y, angle);
	transform_point(&sg->edges[0], x, y, angle);
	transform_point(&sg->edges[1], x, y, angle);

	sg->m = tan(sg->slope);
	sg->n = sg->middle.y - sg->middle.x * sg->m;
}

float slope_dif(float slope0, float slope1)
{
	if(slope0 < slope1)
		return min(slope1 - slope0, slope0 + (float)PI - slope1);
	else
		return min(slope0 -  slope1, slope1 + (float)PI - slope0);
}

void transform_segments(segment_t *sg, int num_sg, float x, float y, float angle)
{
	for (int i = 0; i < num_sg; ++i)
	{
		sg[i].slope += angle;
		// while (sg[i].slope < 0) sg[i].slope += PI;
		// while (sg[i].slope > PI) sg[i].slope -= PI;

		transform_point(&sg[i].middle, x, y, angle);
		transform_point(&sg[i].edges[0], x, y, angle);
		transform_point(&sg[i].edges[1], x, y, angle);

		sg[i].m = tan(sg[i].slope);
		sg[i].n = sg[i].middle.y - sg[i].middle.x * sg[i].m;
	}
}