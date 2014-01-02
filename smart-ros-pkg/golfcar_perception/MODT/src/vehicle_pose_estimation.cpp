#include "vehicle_pose_estimation.h"

namespace mrpt{
	
	vehicle_pose_estimation::vehicle_pose_estimation()
	{
		ros::NodeHandle nh;
		segpose_batch_sub_  = nh.subscribe("segment_pose_batches", 1, &vehicle_pose_estimation::estimate_pose, this);
		shape_pcl_pub_		= nh.advertise<sensor_msgs::PointCloud>("shape_pcl", 2);
		polygon_pub_		= nh.advertise<geometry_msgs::PolygonStamped>("vehicle_polygon", 2);
	}

	void vehicle_pose_estimation::estimate_pose(const MODT::segment_pose_batches& batches)
	{
		ROS_INFO("receive batches");
		if(batches.clusters.size()==0) return;
		sensor_msgs::PointCloud cloud_to_process = batches.clusters.back().segments.back();
		RANSAC_shape(cloud_to_process);
	}

	void vehicle_pose_estimation::RANSAC_shape(sensor_msgs::PointCloud &input_cloud)
	{
		ROS_INFO("point number: %ld", input_cloud.points.size());

		//step1: use RANSAC to get the shape;
		mrpt::dynamicsize_vector<double> xs, ys;
		std::vector<TPoint2D> Tpoint_vector;
		std::vector<geometry_msgs::Point32> Gpoint_vector;
		for(size_t k=0; k<input_cloud.points.size(); k++)
		{
			xs.push_back( input_cloud.points[k].x);
			ys.push_back( input_cloud.points[k].y);

			TPoint2D pttmp;
			pttmp.x = input_cloud.points[k].x;
			pttmp.y = input_cloud.points[k].y;
			Tpoint_vector.push_back(pttmp);

			geometry_msgs::Point32 Gpttmp;
			Gpttmp.x = input_cloud.points[k].x;
			Gpttmp.y = input_cloud.points[k].y;
			Gpoint_vector.push_back(Gpttmp);
		}

		std::vector<std::pair<mrpt::vector_size_t, Lshape> >  Lshape_models;
		ransac_detect_Lshape(xs, ys, Lshape_models, 0.2, 5);
		if(Lshape_models.size()==0)return;

		sensor_msgs::PointCloud shape_cloud;
		shape_cloud.header=input_cloud.header;

		//FYI, actually there is only one model in Lshape_models vector, due to the implementation of RANSAC model;
		for(unsigned int id =0; id < Lshape_models.front().first.size(); id++)
		{
			geometry_msgs::Point32 &pttmp = Gpoint_vector[Lshape_models.front().first[id]];
			shape_cloud.points.push_back(pttmp);
		}
		shape_pcl_pub_.publish(shape_cloud);

		//step2: refine the model;
		//a. associate points with their belonging line;

		double *coefs = Lshape_models.front().second.coefs;
		TLine2D  line1(coefs[0],coefs[1],coefs[2]);
		TLine2D  line2(coefs[1],-coefs[0],coefs[3]);

		TPoint2D intersection_pt; geometry_msgs::Point32 intersection_Gpt;
	 	TObject2D intersection_obj;
		if(intersect(line1, line2, intersection_obj))
		{
			intersection_obj.getPoint(intersection_pt);
			intersection_Gpt.x = (float)intersection_pt.x;
			intersection_Gpt.y = (float)intersection_pt.y;
		}
		else {ROS_WARN("two lines doesn't insect, this cannot be like this;");}

		std::vector<size_t> line1_serial, line2_serial;

		ROS_INFO("coefs[4]: (%lf, %lf, %lf, %lf)", coefs[0],coefs[1],coefs[2],coefs[3]);
		for(unsigned int id =0; id < Lshape_models.front().first.size(); id++)
		{
			TPoint2D &pttmp = Tpoint_vector[Lshape_models.front().first[id]];
			double dist1, dist2;
			dist1 = line1.distance(pttmp);
			dist2 = line2.distance(pttmp);
			if(dist1<dist2) line1_serial.push_back(Lshape_models.front().first[id]);
			else line2_serial.push_back(Lshape_models.front().first[id]);
		}

		//b. split the points in each line into two groups, with each group at the two sides of the other line;
		std::vector<size_t> line1_group1, line1_group2, line2_group1, line2_group2;
		for(size_t i=0; i<line1_serial.size(); i++)
		{
			size_t serial_tmp = line1_serial[i];
			TPoint2D &pttmp = Tpoint_vector[serial_tmp];
			if(line2.evaluatePoint(pttmp)>0)line1_group1.push_back(serial_tmp);
			else line1_group2.push_back(serial_tmp);
		}
		for(size_t i=0; i<line2_serial.size(); i++)
		{
			size_t serial_tmp = line2_serial[i];
			TPoint2D &pttmp = Tpoint_vector[serial_tmp];
			if(line1.evaluatePoint(pttmp)>0)line2_group1.push_back(serial_tmp);
			else line2_group2.push_back(serial_tmp);
		}
		std::vector<size_t> line1_pruned, line2_pruned;
		line1_pruned = line1_group1.size()>line1_group2.size()?line1_group1:line1_group2;
		line2_pruned = line2_group1.size()>line2_group2.size()?line2_group1:line2_group2;

		//check whether these two lines are long enough, to differentiate "L-shape" and "single line";
		size_t line1_farestPt_serial,line2_farestPt_serial;
		double line1_farestPt_dist = 0.0, line2_farestPt_dist=0.0;
		for(size_t i=0; i<line1_pruned.size(); i++)
		{
			size_t serial_tmp = line1_pruned[i];
			TPoint2D &pttmp = Tpoint_vector[serial_tmp];
			double dist_tmp = line2.distance(pttmp);
			if(dist_tmp>line1_farestPt_dist)
			{
				line1_farestPt_serial = serial_tmp;
				line1_farestPt_dist = dist_tmp;
			}
		}
		for(size_t i=0; i<line2_pruned.size(); i++)
		{
			size_t serial_tmp = line2_pruned[i];
			TPoint2D &pttmp = Tpoint_vector[serial_tmp];
			double dist_tmp = line1.distance(pttmp);
			if(dist_tmp>line2_farestPt_dist)
			{
				line2_farestPt_serial = serial_tmp;
				line2_farestPt_dist = dist_tmp;
			}
		}


		geometry_msgs::PolygonStamped vehicle_polygon;
		vehicle_polygon.header = input_cloud.header;
		if(line1_farestPt_dist> SIDE_LENGTH_THRESHOLD && line2_farestPt_dist>SIDE_LENGTH_THRESHOLD)
		{
			ROS_INFO("detect Lshape!");
			vehicle_polygon.polygon.points.push_back(Gpoint_vector[line1_farestPt_serial]);
			vehicle_polygon.polygon.points.push_back(intersection_Gpt);
			vehicle_polygon.polygon.points.push_back(Gpoint_vector[line2_farestPt_serial]);
			vehicle_polygon.polygon.points.push_back(intersection_Gpt);
		}
		else if(line1_farestPt_dist> SIDE_LENGTH_THRESHOLD && line2_farestPt_dist<SIDE_LENGTH_THRESHOLD)
		{
			ROS_INFO("detect Single Line");
			//the intersection point will make no sense, hence need to calculate the two ends of the line;
			TPoint2D &pt_end1 = Tpoint_vector[line1_farestPt_serial];
			double longest_distance_to_end1 =0.0;
			size_t end2_serial;
			for(size_t i=0; i<line1_pruned.size(); i++)
			{
				size_t serial_tmp = line1_pruned[i];
				TPoint2D &pttmp = Tpoint_vector[serial_tmp];
				double dist_tmp = sqrt((pttmp.x-pt_end1.x)*(pttmp.x-pt_end1.x)+(pttmp.y-pt_end1.y)*(pttmp.y-pt_end1.y));
				if(dist_tmp>longest_distance_to_end1)
				{
					end2_serial = serial_tmp;
					longest_distance_to_end1 = dist_tmp;
				}
			}
			//TPoint2D &pt_end2 = Tpoint_vector[end2_serial];


			vehicle_polygon.polygon.points.push_back(Gpoint_vector[line1_farestPt_serial]);
			vehicle_polygon.polygon.points.push_back(Gpoint_vector[end2_serial]);
		}
		else if(line1_farestPt_dist< SIDE_LENGTH_THRESHOLD && line2_farestPt_dist>SIDE_LENGTH_THRESHOLD)
		{
			ROS_INFO("detect Single Line");
			//the intersection point will make no sense, hence need to calculate the two ends of the line;
			TPoint2D &pt_end1 = Tpoint_vector[line2_farestPt_serial];
			double longest_distance_to_end1 =0.0;
			size_t end2_serial;
			for(size_t i=0; i<line2_pruned.size(); i++)
			{
				size_t serial_tmp = line2_pruned[i];
				TPoint2D &pttmp = Tpoint_vector[serial_tmp];
				double dist_tmp = sqrt((pttmp.x-pt_end1.x)*(pttmp.x-pt_end1.x)+(pttmp.y-pt_end1.y)*(pttmp.y-pt_end1.y));
				if(dist_tmp>longest_distance_to_end1)
				{
					end2_serial = serial_tmp;
					longest_distance_to_end1 = dist_tmp;
				}
			}
			//TPoint2D &pt_end2 = Tpoint_vector[end2_serial];

			vehicle_polygon.polygon.points.push_back(Gpoint_vector[line2_farestPt_serial]);
			vehicle_polygon.polygon.points.push_back(Gpoint_vector[end2_serial]);
		}
		else
		{
			ROS_WARN("no reliable size, no reliable output");
		}
		polygon_pub_.publish(vehicle_polygon);
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vehicle_pose_estimation_node");
	mrpt::vehicle_pose_estimation vehicle_pose_estimation_node;
	ros::spin();
	return (0);
}

