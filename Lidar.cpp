
#include <iostream>
#include "Lidar.h"
#include "Detect.h"

using namespace std;

//void processLidar(state_t &s, lidar_t lData)
//{
	//list<feature_t> detectedFeatures;
	//list<feature_t> newFeatures;
	//list<match_t> matches;

	//// Clear all intermediary buffers
	//detectedFeatures.clear();
	//newFeatures.clear();
	//matches.clear();

	//// Ladmark extraction - deteNew Folder1ct features in current data - local MinMax algo
	//extractFeatures(lData, detectedFeatures, s);

	//// Data association
	//// Match current vs previous features
	//// update matches
	//// keep new features
	//matchFeatures(detectedFeatures, matches, newFeatures, s);
	////cout << "matches: " << matches.size() << endl;
	//// EKF
	//EKF(s, matches);
	//removeOldFeatures(newFeatures.size(), s);

	//addNewFeatures(newFeatures, s);

	//cout << "features: " << s.fcount << endl;
//}

int convertLidarToSamplePoint(lidar_t lidar, point_t* points, int max_num_samples)
{
	int index = 0;
	float angle = lidar.start_angle;

	for (int i = 0; i < lidar.num_readings; ++i)
	{
		if (lidar.data[i] >= 20)
			continue;

		point_t new_point;
		new_point.x = lidar.data[i] * cos(angle);
		new_point.y = lidar.data[i] * sin(angle);
		points[index++] = new_point;

		angle += lidar.angular_resolution;
	}

	return index;
}