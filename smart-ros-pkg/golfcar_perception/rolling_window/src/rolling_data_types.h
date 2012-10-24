#ifndef ROLLING_DATA_TYPES
#define ROLLING_DATA_TYPES

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct RollingPointXYZ
{
	PCL_ADD_POINT4D;

	//reserve useful information when accumulated laser points;
	unsigned int laser_serial;
	float beam_angle;
	float x_var, y_var, z_var;
	float pitch_speed;
	float roll_speed;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment



struct  RollingPointXYZNormal
{
	PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])

	//reserve useful information when accumulated laser points;
	unsigned int laser_serial;
	float beam_angle;
	float x_var, y_var, z_var;
	float pitch_speed;
	float roll_speed;

	PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
	union
	{
		struct
		{
		  float curvature;
		};
		float data_c[4];
	};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

 POINT_CLOUD_REGISTER_POINT_STRUCT (RollingPointXYZ,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (unsigned int, laser_serial, laser_serial)
                                    (float, beam_angle, beam_angle)
                                    (float, x_var, x_var)
                                    (float, y_var, y_var)
                                    (float, z_var, z_var)
                                    (float, pitch_speed, pitch_speed)
                                    (float, roll_speed, roll_speed)
 )

 POINT_CLOUD_REGISTER_POINT_STRUCT (RollingPointXYZNormal,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(unsigned int, laser_serial, laser_serial)
	(float, beam_angle, beam_angle)
	(float, x_var, x_var)
	(float, y_var, y_var)
	(float, z_var, z_var)
	(float, normal_x, normal_x)
	(float, normal_y, normal_y)
	(float, normal_z, normal_z)
	(float, curvature, curvature)
	(float, pitch_speed, pitch_speed)
	(float, roll_speed, roll_speed)
)

#endif
