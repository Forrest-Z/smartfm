vector<geometry_msgs::Point32> getTransformedPts(geometry_msgs::Point32 pose, vector<geometry_msgs::Point32>& pts)
{
	double ct = cos(pose.z), st = sin(pose.z);
	vector<geometry_msgs::Point32> final_pt;
	final_pt.resize(pts.size());
	for(size_t j=0; j<pts.size(); j++)
	{

		geometry_msgs::Point32 pt = pts[j], rot_pt;
		rot_pt.x = ct * pt.x - st * pt.y + pose.x;
		rot_pt.y = st * pt.x + ct * pt.y + pose.y;
		final_pt[j] = rot_pt;
	}
	return final_pt;
}

vector<geometry_msgs::Point32> getTransformedPts(geometry_msgs::Pose pose, vector<geometry_msgs::Point32>& pts)
		{
	double ct = cos(pose.orientation.z), st = sin(pose.orientation.z);
	vector<geometry_msgs::Point32> final_pt;
	final_pt.resize(pts.size());
	for(size_t j=0; j<pts.size(); j++)
	{

		geometry_msgs::Point32 pt = pts[j], rot_pt;
		rot_pt.x = ct * pt.x - st * pt.y + pose.position.x;
		rot_pt.y = st * pt.x + ct * pt.y + pose.position.y;
		rot_pt.z = pt.z;
		final_pt[j] = rot_pt;
	}
	return final_pt;
		}
geometry_msgs::Point32 ominus(geometry_msgs::Point32 point2, geometry_msgs::Point32 point1)
{
	double ctheta = cos(point1.z), stheta = sin(point1.z);
	geometry_msgs::Point32 relative_tf;
	relative_tf.x  = (point2.x - point1.x) * ctheta + (point2.y - point1.y) * stheta;
	relative_tf.y =  -(point2.x - point1.x) * stheta + (point2.y - point1.y) * ctheta;
	relative_tf.z = point2.z - point1.z;
	//cout<<relative_tf<<endl;
	return relative_tf;
}

geometry_msgs::Pose ominus(geometry_msgs::Pose point2, geometry_msgs::Pose point1)
{
	double ctheta = cos(point1.orientation.z), stheta = sin(point1.orientation.z);
	geometry_msgs::Pose relative_tf;
	relative_tf.position.x  = (point2.position.x - point1.position.x) * ctheta + (point2.position.y - point1.position.y) * stheta;
	relative_tf.position.y =  -(point2.position.x - point1.position.x) * stheta + (point2.position.y - point1.position.y) * ctheta;
	relative_tf.position.z = point2.position.z - point1.position.z;
	relative_tf.orientation.z = point2.orientation.z - point1.orientation.z;
	relative_tf.orientation.y = point2.orientation.y - point1.orientation.y;
	//cout<<relative_tf<<endl;
	return relative_tf;
}
