#include <vector>

namespace geometry_msgs
{
	struct Point32
	{
		double x, y, z;
	};

	struct Pose
	{
		Point32 position;
		Point32 orientation;
	};
}
namespace sensor_msgs
{
	struct PointCloud
	{
		std::vector<geometry_msgs::Point32> points;
	};
}
