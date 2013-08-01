dbgstream dbgbuf;
bool readPts3D(istream &in, geometry_msgs::Pose &pose)
{
	if(!(in >> pose.position.x )) return false;
	if(!(in >> pose.position.y)) return false;
	if(!(in >> pose.position.z)) return false;
	if(!(in >> pose.orientation.x)) return false;
	if(!(in >> pose.orientation.y)) return false;
	if(!(in >> pose.orientation.z)) return false;
  return true;
}

bool readFrontEndFile(const char* filename, vector<geometry_msgs::Pose>& poses)
{
  poses.clear();
	geometry_msgs::Pose pose;
  istream*        data_in         = NULL;         // input for data points
		ifstream dataStreamSrc;
		dataStreamSrc.open(filename, ios::in);// open data file
		if (!dataStreamSrc) {
			cerr << "Cannot open data file\n";
      cerr << "Given: "<<filename<<endl;
			exit(1);
		}
		data_in = &dataStreamSrc;
	while(readPts3D(*data_in, pose))
	{
		poses.push_back(pose);
	}
	
	dbgbuf<<"Successfully read "<<poses.size()<<" vectors of poses"<<endl;
	return true;
}
struct simple2D{
  int x;
  int y;
  int index;
};
struct Simple2D{
  double x, y, ang;
  double score;
};
bool data_sort(simple2D s1, simple2D s2){
  return s1.index < s2.index;
}
void filterPointCloudByGrid(pcl::PointCloud<pcl::PointNormal> &matching_cloud,
                            cv::Mat &template_map,
  double res, int offset_x, int offset_y, vector<simple2D> &filtered_data,
  vector<int> &filtered_data_count
){
  float score = 0;
  int data_size = matching_cloud.points.size();
  vector<simple2D> data(data_size);
  for(int i=0; i<data_size; i++){
    data[i].x = round(matching_cloud.points[i].x/res)+offset_x;
    data[i].y = round(matching_cloud.points[i].y/res)+offset_y;
    if(data[i].x < offset_x*2 && data[i].y < offset_y*2){
      int ptr = data[i].x + data[i].y*template_map.cols;
      score+=template_map.data[ptr]/255.f;
      data[i].index = ptr;
    }
    else
      data[i].index = template_map.cols*template_map.rows;
    
  }
  sort(data.begin(), data.end(), data_sort);
  cout<<"Gridded "<<" cpu score check = "<<score/data_size<<endl;
  
  int last_idx = data[0].index;
  int data_count = 0;
  int raw_data_x[data_size];
  int raw_data_y[data_size];
  for(int i=0; i<data_size; i++){
    raw_data_x[i] = data[i].x;
    raw_data_y[i] = data[i].y;
    data_count++;
    if(data[i].index != last_idx){
      filtered_data.push_back(data[i-data_count+1]);
      filtered_data_count.push_back(data_count);
      last_idx = data[i].index;
      data_count= 0;
    }
  }
  int raw_input_data_count[data_size];
  for(int i=0; i<data_size; i++) raw_input_data_count[i]=1;
  filtered_data.push_back(data[data.size()-data_count]);
  filtered_data_count.push_back(data_count);
  
}
vector<pcl::PointCloud<pcl::PointNormal> > append_input_cloud(pcl::PointCloud<pcl::PointNormal>& input_cloud, string front_end_str, string input_cloud_str)
{
  vector<geometry_msgs::Pose> poses;
  vector<pcl::PointCloud<pcl::PointNormal> > input_clouds;
  input_clouds.push_back(input_cloud);
  readFrontEndFile(front_end_str.c_str(), poses);
  string input_idx_str = input_cloud_str, folder_str = input_cloud_str;
  input_idx_str = input_idx_str.substr(input_idx_str.size()-9, 5);
  folder_str = folder_str.substr(0, folder_str.size()-9);
  int input_idx = atoi(input_idx_str.c_str());
  dbgbuf<<"input_idx="<<input_idx<<endl;
  map<int, geometry_msgs::Pose> poses_map; 
  //map<int, pcl::PointCloud<pcl::PointNormal> > input_cloud_map;
  int start_append = -4, end_append = 6;
  for(int i=start_append; i<end_append; i++)
  {
    poses_map[i] = poses[input_idx+i];
  }
  //dbgbuf<<"Given 0 pose "<<poses_map[0].position.x<<" "<<poses_map[0].position.y<<" "<<poses_map[0].orientation.z/M_PI*180<<endl;
  for(int i=start_append; i<end_append; i++)
  {
    if(i==0) continue;
    stringstream add_input_ss;
    add_input_ss<<folder_str<<setfill('0')<<setw(5)<<input_idx+i<<".pcd";
    dbgbuf<<"Reading add_input file: "<<add_input_ss.str()<<endl;
    pcl::PointCloud<pcl::PointNormal> add_input_cloud;
    pcl::io::loadPCDFile(add_input_ss.str(), add_input_cloud);
    geometry_msgs::Pose rel_pose = ominus( poses_map[i], poses_map[0]); 
    dbgbuf<<"Given "<<i<<" pose "<<poses_map[i].position.x<<" "<<poses_map[i].position.y<<" "<<poses_map[i].orientation.z/M_PI*180<<endl;
    dbgbuf<<"Relative Orientation in deg: "<<rel_pose.orientation.z/M_PI*180<<endl;
    dbgbuf<<"Relative Translation: "<<rel_pose.position.x<<" "<<rel_pose.position.y<<endl;
		double yaw_rotate = -rel_pose.orientation.z;
		Eigen::Vector3f bl_trans(rel_pose.position.x, rel_pose.position.y, 0.);
		Eigen::Quaternionf bl_rotation (cos(yaw_rotate/2.), 0, 0, -sin(yaw_rotate/2.) );
		pcl::transformPointCloudWithNormals<pcl::PointNormal>(add_input_cloud, add_input_cloud, bl_trans, bl_rotation);
    input_clouds.push_back(add_input_cloud);  
    input_cloud += add_input_cloud;
  }
  return input_clouds;
}