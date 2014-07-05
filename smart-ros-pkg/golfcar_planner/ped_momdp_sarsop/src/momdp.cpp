#include "momdp.h"
#include "node.h"
#include "solver.h"
#include "globals.h"

double marker_colors[20][3] = {
	{0.0,1.0,0.0},
		{1.0,0.0,0.0},
		{0.0,0.0,1.0},
		{1.0,1.0,0.0},
		{0.0,1.0,1.0},
		{1.0,0.0,1.0},
		{0.0,0.0,0.0}
};

int action_map[3]={2,0,1};
int WorldSeed() {
  cout<<"root seed"<<Globals::config.root_seed<<endl;
  return Globals::config.root_seed ^ Globals::config.n_particles;
}

int BeliefUpdateSeed() {
  return Globals::config.root_seed ^ (Globals::config.n_particles + 1);
}

// Action selection for RandomPolicyLowerBound (if used):
int RandomActionSeed() {
  return Globals::config.root_seed ^ (Globals::config.n_particles + 2);
}

ped_momdp::ped_momdp(ros::NodeHandle& nh, bool fixed_path, double pruning_constant):  worldStateTracker(worldModel), worldBeliefTracker(worldModel, worldStateTracker), fixed_path_(fixed_path)
{
	cout << "fixed_path = " << fixed_path_ << endl;
	Globals::config.pruning_constant = pruning_constant;

    nh.param("pruning_constant", Globals::config.pruning_constant, 0.0);

	global_frame_id = ModelParams::rosns + "/map";
	cerr <<"DEBUG: Entering ped_momdp()"<<endl;
	control_freq=ModelParams::control_freq;

	cerr << "DEBUG: Initializing publishers..." << endl;
    believesPub_ = nh.advertise<ped_momdp_sarsop::peds_believes>("peds_believes",1);
    cmdPub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_pomdp",1);
	actionPub_ = nh.advertise<visualization_msgs::Marker>("pomdp_action",1);
	pathPub_= nh.advertise<nav_msgs::Path>("pomdp_path_repub",1, true);
	pathSub_= nh.subscribe("plan", 1, &ped_momdp::RetrievePathCallBack, this);
	goal_pub=nh.advertise<visualization_msgs::MarkerArray> ("pomdp_goals",1);
	start_goal_pub=nh.advertise<ped_pathplan::StartGoal> ("ped_path_planner/planner/start_goal", 1);

	markers_pub=nh.advertise<visualization_msgs::MarkerArray>("pomdp_belief",1);
	safeAction=2;
	momdp_speed_=0.0;
	goal_reached=false;
	cerr << "DEBUG: Init simulator" << endl;
	initSimulator();
    //RetrievePaths();

	cerr <<"DEBUG: before entering controlloop"<<endl;
    timer_ = nh.createTimer(ros::Duration(1.0/control_freq), &ped_momdp::controlLoop, this);
	timer_speed=nh.createTimer(ros::Duration(0.05), &ped_momdp::publishSpeed, this);

}



bool ped_momdp::getObjectPose(string target_frame, tf::Stamped<tf::Pose>& in_pose, tf::Stamped<tf::Pose>& out_pose) const
{
    out_pose.setIdentity();

    try {
        tf_.transformPose(target_frame, in_pose, out_pose);
    }
    catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        return false;
    }
    return true;
}


/*for despot*/
void ped_momdp::initSimulator()
{
  Globals::config.root_seed=1024;
  //Globals::config.n_belief_particles=2000;
  Globals::config.n_particles=100;
  Globals::config.time_per_move = 1.0/ModelParams::control_freq;
  Seeds::root_seed(Globals::config.root_seed);
  cerr << "Random root seed set to " << Globals::config.root_seed << endl;

  // Global random generator
  double seed = Seeds::Next();
  Random::RANDOM = Random(seed);
  cerr << "Initialized global random generator with seed " << seed << endl;

  despot=new PedPomdp(worldModel); 
  despot->num_active_particles = 0;

  RandomStreams* streams = NULL;
  streams = new RandomStreams(Seeds::Next(Globals::config.n_particles), Globals::config.search_depth);
  despot->InitializeParticleLowerBound("smart");
  despot->InitializeScenarioLowerBound("smart", *streams);
  despot->InitializeParticleUpperBound("smart", *streams);
  despot->InitializeScenarioUpperBound("smart", *streams);
  solver = new DESPOTSTAR(despot, NULL, *streams);
}



ped_momdp::~ped_momdp()
{
    geometry_msgs::Twist cmd;
    cmd.angular.z = 0;
    cmd.linear.x = 0;
    cmdPub_.publish(cmd);
}



void ped_momdp::updateSteerAnglePublishSpeed(geometry_msgs::Twist speed)
{
    //cmdPub_.publish(speed);
}

void ped_momdp::publishSpeed(const ros::TimerEvent &e)
{
	geometry_msgs::Twist cmd;
	cmd.angular.z = 0;       	
	cmd.linear.x = momdp_speed_;
	cout<<"publishing cmd speed "<<momdp_speed_<<endl;
	cmdPub_.publish(cmd);
}



void ped_momdp::publishROSState()
{
	geometry_msgs::Point32 pnt;
	
	geometry_msgs::Pose pose;
	
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header.stamp=ros::Time::now();

	pose_stamped.header.frame_id=global_frame_id;

	pose_stamped.pose.position.x=(worldStateTracker.carpos.x+0.0);
	pose_stamped.pose.position.y=(worldStateTracker.carpos.y+0.0);
	pose_stamped.pose.orientation.w=1.0;
	car_pub.publish(pose_stamped);
	
	geometry_msgs::PoseArray pA;
	pA.header.stamp=ros::Time::now();

	pA.header.frame_id=global_frame_id;
	for(int i=0;i<worldStateTracker.ped_list.size();i++)
	{
		//GetCurrentState(ped_list[i]);
		pose.position.x=(worldStateTracker.ped_list[i].w+0.0);
		pose.position.y=(worldStateTracker.ped_list[i].h+0.0);
		pose.orientation.w=1.0;
		pA.poses.push_back(pose);
		//ped_pubs[i].publish(pose);
	}

	pa_pub.publish(pA);


	uint32_t shape = visualization_msgs::Marker::CYLINDER;

	for(int i=0;i<worldModel.goals.size();i++)
	{
		visualization_msgs::Marker marker;

		marker.header.frame_id=ModelParams::rosns+"/map";
		marker.header.stamp=ros::Time::now();
		marker.ns="basic_shapes";
		marker.id=i;
		marker.type=shape;
		marker.action = visualization_msgs::Marker::ADD;



		marker.pose.position.x = worldModel.goals[i].x;
		marker.pose.position.y = worldModel.goals[i].y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		marker.scale.x = 1;
		marker.scale.y = 1;
		marker.scale.z = 1;
		marker.color.r = marker_colors[i][0];
		marker.color.g = marker_colors[i][1];
		marker.color.b = marker_colors[i][2];
		marker.color.a = 1.0;
		
		markers.markers.push_back(marker);
	}
	goal_pub.publish(markers);
	markers.markers.clear();
}




void ped_momdp::publishAction(int action)
{
		uint32_t shape = visualization_msgs::Marker::CUBE;
		visualization_msgs::Marker marker;			
		
		marker.header.frame_id=global_frame_id;
		marker.header.stamp=ros::Time::now();
		marker.ns="basic_shapes";
		marker.id=0;
		marker.type=shape;
		marker.action = visualization_msgs::Marker::ADD;

		double px,py;
		px=worldStateTracker.carpos.x;
		py=worldStateTracker.carpos.y;
		marker.pose.position.x = px+1;
		marker.pose.position.y = py+1;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 0.6;
		marker.scale.y = 3;
		marker.scale.z = 0.6;
		//
		// Set the color -- be sure to set alpha to something non-zero!
		//marker.color.r = 0.0f;
		//marker.color.g = 1.0f;
		//marker.color.b = 0.0f;
		//marker.color.a = 1.0;
		marker.color.r = marker_colors[action_map[action]][0];
        marker.color.g = marker_colors[action_map[action]][1];
		marker.color.b = marker_colors[action_map[action]][2];//marker.lifetime = ros::Duration();
		marker.color.a = 1.0;

		ros::Duration d(1/control_freq);
		marker.lifetime=d;
		actionPub_.publish(marker);		

}
//for despot

void ped_momdp::RetrievePaths(const tf::Stamped<tf::Pose>& carpose)
{
	//RealSimulator->global_path[];	
	
	//pomdp_path_planner::GetPomdpPath srv;	
	//path_client.call(srv);
	//int size=srv.response.CurrPaths.size();
	//cout<<"path size "<<size<<endl;
	//cout<<"first path"<<endl;
	//if(size!=0)
	//{
	//	pomdp_path_planner::PomdpPath path=srv.response.CurrPaths[0];
	//	for(int i=0;i<size;i++)
	//		cout<<path.points[i].x<<" "<<path.points[i].y<<endl;
	//}
	//

	if(fixed_path_ && worldModel.path.size()>0)  return;

	ped_pathplan::StartGoal startGoal; 
	nav_msgs::GetPlan srv;
	geometry_msgs::PoseStamped pose;
	tf::poseStampedTFToMsg(carpose, pose);

	//pose.header.stamp=ros::Time::now();
	//pose.header.frame_id=global_frame_id;
	//pose.pose.position.x=worldStateTracker.carpos.x;
	//pose.pose.position.y=worldStateTracker.carpos.y;

	//srv.request.start=pose;
	startGoal.start=pose;

	// set goal
	//for simulation
	pose.pose.position.x=17;
	pose.pose.position.y=52;
	//for utown 
	
	//pose.pose.position.x=108;
	//pose.pose.position.y=143;
	//srv.request.tolerance=1.0;
	//srv.request.goal=pose;
	startGoal.goal=pose;
	start_goal_pub.publish(startGoal);	

//	path_client.call(srv);

	//nav_msgs::Path p;
	//p.header.stamp=ros::Time::now();
	//p.poses=srv.response.plan;

}

void ped_momdp::RetrievePathCallBack(const nav_msgs::Path::ConstPtr path)  {
	cout<<"receive path from navfn "<<path->poses.size()<<endl;
	if(fixed_path_ && worldModel.path.size()>0) return;

	if(path->poses.size()==0) return;
	Path p;
	for(int i=0;i<path->poses.size();i++)
	{
        COORD coord;
		coord.x=path->poses[i].pose.position.x;
		coord.y=path->poses[i].pose.position.y;
        p.push_back(coord);
	}
	worldModel.setPath(p.interpolate());
	pathPub_.publish(*path);
}

void ped_momdp::controlLoop(const ros::TimerEvent &e)
{

	    cout<<"entering control loop"<<endl;
        tf::Stamped<tf::Pose> in_pose, out_pose;

        ros::Rate err_retry_rate(10);

		//transpose to base link
		in_pose.setIdentity();
		in_pose.frame_id_ = ModelParams::rosns + "/base_link";
		while(!getObjectPose(global_frame_id, in_pose, out_pose)) {
			cerr<<"transform error within control loop"<<endl;
			cout<<"laser frame "<<in_pose.frame_id_<<endl;
            err_retry_rate.sleep();
		}

		RetrievePaths(out_pose);
		if(worldModel.path.size()==0) return;
		//transpose to laser frame
		in_pose.setIdentity();
		in_pose.frame_id_ = ModelParams::rosns + ModelParams::laser_frame;
		while(!getObjectPose(global_frame_id, in_pose, out_pose)) {
			cerr<<"transform error within control loop"<<endl;
			cout<<"laser frame "<<in_pose.frame_id_<<endl;
            err_retry_rate.sleep();
		}

		COORD coord;
		coord.x=out_pose.getOrigin().getX();
		coord.y=out_pose.getOrigin().getY();
		cout << "transformed pose = " << coord.x << " " << coord.y << endl;
		worldStateTracker.updateCar(coord);

        worldStateTracker.cleanPed();

		PomdpState curr_state = worldStateTracker.getPomdpState();
		publishROSState();
		despot->PrintState(curr_state, cout);
		cout<<"here"<<endl;
		if(worldModel.isGlobalGoal(curr_state.car))
		{
			goal_reached=true;
		}
		if(goal_reached==true)
		{
			safeAction=2;

			momdp_speed_=real_speed_;
		    momdp_speed_ -= 0.5;
			if(momdp_speed_<=0.0) momdp_speed_ = 0.0;

            // shutdown the node after reaching goal
            // TODO consider do this in simulaiton only for safety
            if(real_speed_ <= 1e-5) {
                ros::shutdown();
            }
			return;
		}

		cout<<"before belief update"<<endl;
		worldBeliefTracker.update();
		cout<<"after belief update"<<endl;
		vector<PomdpState> samples = worldBeliefTracker.sample(1000);
		vector<State*> particles = despot->ConstructParticles(samples);

		double sum=0;
		for(int i=0;i<particles.size();i++)
			sum+=particles[i]->weight;
		cout<<"particle weight sum "<<sum<<endl;
		ParticleBelief *pb=new ParticleBelief(particles, despot);
		solver->belief(pb);

		if(worldStateTracker.emergency())
		{
			momdp_speed_=-1;
			return;
		}


		safeAction=solver->Search();

		//actionPub_.publish(action);
		publishAction(safeAction);

		cout<<"safe action "<<safeAction<<endl;


		publishBelief();

		momdp_speed_=real_speed_;
        if(safeAction==0) {}
		else if(safeAction==1) momdp_speed_ += 0.3*2;
		else if(safeAction==2) momdp_speed_ -= 0.5*2;
		if(momdp_speed_<=0.0) momdp_speed_ = 0.0;
		if(momdp_speed_>=ModelParams::VEL_MAX) momdp_speed_ = ModelParams::VEL_MAX;

		cout<<"momdp_speed "<<momdp_speed_<<endl;
		delete pb;	
}


void ped_momdp::publishMarker(int id,PedBelief & ped)
{
	//cout<<"belief vector size "<<belief.size()<<endl;
	std::vector<double> belief = ped.prob_goals;
	uint32_t shape = visualization_msgs::Marker::CUBE;
	for(int i=0;i<belief.size();i++)
	{
		visualization_msgs::Marker marker;			

		marker.header.frame_id=global_frame_id;
		marker.header.stamp=ros::Time::now();
		marker.ns="basic_shapes";
		marker.id=id*ped.prob_goals.size()+i;
		marker.type=shape;
		marker.action = visualization_msgs::Marker::ADD;

		double px=0,py=0;
		px=ped.pos.x;
		py=ped.pos.y;
		marker.pose.position.x = px+i*0.7;
		marker.pose.position.y = py+belief[i]*2;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
	//	cout<<"belief entries "<<px<<" "<<py<<endl;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 0.5;
		marker.scale.y = belief[i]*4;
		//if(marker.scale.y<0.2) marker.scale.y=0.2;
		marker.scale.z = 0.2;
		//
		// Set the color -- be sure to set alpha to something non-zero!
		//marker.color.r = 0.0f;
		//marker.color.g = 1.0f;
		//marker.color.b = 0.0f;
		//marker.color.a = 1.0;
		marker.color.r = marker_colors[i][0];
        marker.color.g = marker_colors[i][1];
		marker.color.b = marker_colors[i][2];//marker.lifetime = ros::Duration();
		marker.color.a = 1.0;

		ros::Duration d(1/control_freq);
		marker.lifetime=d;
		markers.markers.push_back(marker);
	}
}
void ped_momdp::publishBelief()
{
	//vector<vector<double> > ped_beliefs=RealSimulator->GetBeliefVector(solver->root_->particles());	
	//cout<<"belief vector size "<<ped_beliefs.size()<<endl;
	int i=0;
	ped_momdp_sarsop::peds_believes pbs;	
	for(auto & kv: worldBeliefTracker.peds)
	{
		publishMarker(i++,kv.second);
		ped_momdp_sarsop::ped_belief pb;
		PedBelief belief = kv.second;	
		pb.ped_x=belief.pos.x;
		pb.ped_y=belief.pos.y;
		for(auto & v : belief.prob_goals)
			pb.belief_value.push_back(v);
		pbs.believes.push_back(pb);
	}
	pbs.cmd_vel=worldStateTracker.carvel;
	pbs.robotx=worldStateTracker.carpos.x;
	pbs.roboty=worldStateTracker.carpos.y;
	believesPub_.publish(pbs);
	markers_pub.publish(markers);
	markers.markers.clear();
}
