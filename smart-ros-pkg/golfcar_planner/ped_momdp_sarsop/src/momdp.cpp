#include "momdp.h"
#include "belief_update/belief_update_particle.h"
#include "vnode.h"
#include "solver.h"
#include "globals.h"
#include "problems/pedestrian_changelane/pedestrian_changelane.h"

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

ped_momdp::ped_momdp(string model_file, string policy_file, int simLen, int simNum, bool stationary, double frequency, bool use_sim_time, ros::NodeHandle& nh) : worldBeliefTracker(worldModel) 
{
	cerr <<"DEBUG: Entering ped_momdp()"<<endl;
	control_freq=frequency;

	cerr << "DEBUG: Initializing publishers..." << endl;
    momdp_problem_timeout = 5.0;
    stationary_ = stationary;
    use_sim_time_ = use_sim_time;
    believesPub_ = nh.advertise<ped_momdp_sarsop::peds_believes>("peds_believes",1);
    cmdPub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_pomdp",1);
	actionPub_ = nh.advertise<visualization_msgs::Marker>("pomdp_action",1);
	pathPub_= nh.advertise<nav_msgs::Path>("pomdp_path_repub",1);
	char buf[100];
	for(int i=0;i<ModelParams::N_PED_IN;i++) {
		sprintf(buf,"pomdp_beliefs%d",i);
		markers_pubs[i]=nh.advertise<visualization_msgs::MarkerArray>(buf,1);
	}
    //initPedGoal();
    //policy_initialize(model_file, policy_file, simLen, simNum);
	safeAction=2;
	momdp_speed_=0.0;
	goal_reached=false;
	cerr << "DEBUG: Init simulator" << endl;
	initRealSimulator();
    RetrievePath();

	cerr <<"frequency "<<frequency<<endl;
	cerr <<"DEBUG: before entering controlloop"<<endl;
  	timer_ = nh.createTimer(ros::Duration(1.0/frequency), &ped_momdp::controlLoop, this);
	timer_speed=nh.createTimer(ros::Duration(0.05), &ped_momdp::publishSpeed, this);

	//initRealSimulator();

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
void ped_momdp::initRealSimulator()
{
  Globals::config.root_seed=1024;
  Globals::config.n_belief_particles=2000;
  Globals::config.n_particles=500;

  cout<<"global particle "<<Globals::config.n_particles<<endl;
  cout<<"root seed "<<Globals::config.root_seed<<endl;
  cout<<"search depth"<<Globals::config.search_depth<<endl;

  ifstream fin("despot.config");
  fin >> Globals::config.pruning_constant;
  cerr << "Pruning constant = " << Globals::config.pruning_constant << endl;

  RealSimulator  =  new Model<PedestrianState>(streams, "pedestrian.config");

  RandomStreams* streams = NULL;
  streams = new RandomStreams(Seeds::Next(Globals::config.n_particles), Globals::config.search_depth);
  PomdpModel.InitializeParticleLowerBound(options[E_BLBTYPE].arg);
  PomdpModel.InitializeScenarioLowerBound(options[E_LBTYPE].arg, *streams);
  PomdpModel.InitializeParticleUpperBound(options[E_BUBTYPE].arg);
  PomdpModel.InitializeScenarioUpperBound(options[E_UBTYPE].arg, *streams);
  solver = new DESPOTSTAR(&PomdpModel, NULL, *streams);



  bu = new ParticleFilterUpdate<PedestrianState>(BeliefUpdateSeed(), *RealSimulator);

  // int ret = Run(model, lb, ub, bu, streams);
  VNode<PedestrianState>::set_model(*RealSimulator);
  int action;
  int i;

  PomdpState ped_state=worldStateTracker.getPomdpState();

  cout<<"start state"<<endl;
  RealSimulator->PrintState(ped_state);

  RealSimulator->SetStartState(ped_state);
  solver=new Solver<PedestrianState>(*RealSimulator, RealSimulator->InitialBelief(), *lb, *RealSimulator, *bu, streams);	
  solver->Init();
  int n_trials;
  solver->Search(1.0/control_freq,n_trials);

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
	double rln=ModelParams::map_rln;
	geometry_msgs::Point32 pnt;
	
	char buf[100];
	sprintf(buf,"%s%s",ModelParams::rosns,"/map");
	

	geometry_msgs::Pose pose;
	
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header.stamp=ros::Time::now();

	sprintf(buf,"%s%s",ModelParams::rosns,"/map");
	pose_stamped.header.frame_id=buf;

	pose_stamped.pose.position.x=(worldStateTracker.car.pos.x+0.0)/rln;
	pose_stamped.pose.position.y=(worldStateTracker.car.pos.y+0.0)/rln;
	pose_stamped.pose.orientation.w=1.0;
	car_pub.publish(pose_stamped);
	
	geometry_msgs::PoseArray pA;
	pA.header.stamp=ros::Time::now();

	sprintf(buf,"%s%s",ModelParams::rosns,"/map");
	pA.header.frame_id=buf;
	for(int i=0;i<worldStateTracker.ped_list.size();i++)
	{
		//GetCurrentState(ped_list[i]);
		pose.position.x=(worldStateTracker.ped_list[i].w+0.0)/rln;
		pose.position.y=(worldStateTracker.ped_list[i].h+0.0)/rln;
		pose.orientation.w=1.0;
		pA.poses.push_back(pose);
		//ped_pubs[i].publish(pose);
	}

	pa_pub.publish(pA);
	ros::Rate loop_rate(1);
	//loop_rate.sleep();
}



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

void ped_momdp::publishAction(int action)
{
		uint32_t shape = visualization_msgs::Marker::CUBE;
		visualization_msgs::Marker marker;			
		
		char buf[100];
		sprintf(buf,"%s%s",ModelParams::rosns,"/map");
		marker.header.frame_id=buf;
		marker.header.stamp=ros::Time::now();
		marker.ns="basic_shapes";
		marker.id=0;
		marker.type=shape;
		marker.action = visualization_msgs::Marker::ADD;

		double px,py;
		px=worldStateTracker.car.pos.x/ModelParams::map_rln;
		py=worldStateTracker.car.pos.y/ModelParams::map_rln;
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

void ped_momdp::RetrievePaths()
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
	nav_msgs::GetPlan srv;
	geometry_msgs::PoseStamped pose;
	pose.header.stamp=ros::Time::now();
	pose.header.frame_id="/map";
	
	pose.pose.position.x=worldStateTracker.car.pos.x;
	pose.pose.position.y=worldStateTracker.car.pos.y;
	srv.request.start=pose;
	pose.pose.position.x=18;
	pose.pose.position.y=49;
	srv.request.tolerance=1.0;
	srv.request.goal=pose;
	path_client.call(srv);
	//nav_msgs::Path p;
	//p.header.stamp=ros::Time::now();
	//p.poses=srv.response.plan;
	cout<<"receive path from navfn "<<srv.response.plan.poses.size()<<endl;
    Path p;
	for(int i=0;i<srv.response.plan.poses.size();i++)
	{
        COORD coord;
		coord.x=srv.response.plan.poses[i].pose.position.x;
		coord.y=srv.response.plan.poses[i].pose.position.y;
        p.push_back(coord);
	}
    stateModel.setPath(p);
	pathPub_.publish(srv.response.plan);
}

void ped_momdp::updatePedPoseReal(PedStruct ped) { 
    for(int i=0;i<ped_list.size();i++)
    {
        if(ped_list[i].id==ped.id)
        {
            //found the corresponding ped,update the pose
            ped_list[i].w=ped.w;
            ped_list[i].h=ped.h;
            ped_list[i].last_update=t_stamp;
            break;
        }
        if(abs(ped_list[i].w-ped.w)<=1&&abs(ped_list[i].h-ped.h)<=1)   //overladp 
            return;
        //		cout<<ped_list[i].w<<" "<<ped_list[i].h<<endl;
    }
    if(i==ped_list.size())   //not found, new ped
    {
        //		cout<<"add"<<endl;
        ped.last_update=t_stamp;
        ped_list.push_back(ped);

    }
}

int brake_counts=0;
void ped_momdp::controlLoop(const ros::TimerEvent &e)
{
	    cout<<"entering control loop"<<endl;

        tf::Stamped<tf::Pose> in_pose, out_pose;
		in_pose.setIdentity();

		char buf[100];
		sprintf(buf,"%s%s",ModelParams::rosns,ModelParams::laser_frame);
		in_pose.frame_id_ = buf; 
		sprintf(buf,"%s%s",ModelParams::rosns,"/map");
		if(!getObjectPose(buf, in_pose, out_pose)) {
			cerr<<"transform error within control loop"<<endl;
		} else {
            COORD coord;
			coord.x=out_pose.getOrigin().getX()*ModelParams::map_rln;
			coord.y=out_pose.getOrigin().getY()*ModelParams::map_rln;
			worldStateTracker.updateCar(coord);
		}
		
		publishROSState();
		
		if(stateModel.isGlobalGoal(worldStateTracker.car))
		{
			goal_reached=true;
		}
		if(goal_reached==true)
		{
			safeAction=2;

			momdp_speed_=real_speed_;
			if(safeAction==0) momdp_speed_ += 0;
			else if(safeAction==1) momdp_speed_ += 0.3;
			else if(safeAction==2) momdp_speed_ -= 0.5;
			if(momdp_speed_<=0.0) momdp_speed_ = 0.0;
			//if(momdp_speed_>=2.0) momdp_speed_ = 2.0;

			if(momdp_speed_>=ModelParams::VEL_MAX) momdp_speed_ = ModelParams::VEL_MAX;

			return;
		}

		RetrievePaths();
		cout<<"State before shift window"<<endl;
		
        worldStateTracker.cleanPed();


		double reward;
		solver->UpdateBelief(safeAction, ped_state,new_state_old);
	
		////////////////////////////////////////////////////////////////////

		if(worldTracker.emergency())
		{
			momdp_speed_=-1;	
			return;
		}

		
		int n_trials;


		safeAction=solver->Search(1.0/control_freq,n_trials);
		//safeAction=solver->Search(0.01,n_trials);

		//actionPub_.publish(action);
		publishAction(safeAction);

		cout<<"n trials "<<n_trials<<endl;
		
		cout<<"safe action "<<safeAction<<endl;
	
		
		publishBelief();

		momdp_speed_=real_speed_;
        if(safeAction==0) {}
		else if(safeAction==1) momdp_speed_ += 0.3*2;
		else if(safeAction==2) momdp_speed_ -= 0.5*2;
		if(momdp_speed_<=0.0) momdp_speed_ = 0.0;
		if(momdp_speed_>=ModelParams::VEL_MAX) momdp_speed_ = ModelParams::VEL_MAX;

		/*
		momdp_speed_=real_speed_;
        if(safeAction==1) {
			if(momdp_speed_ < 0.6) {
				momdp_speed_ = 1.0;
			} else {
				momdp_speed_ += 0.5;
			}
		}
        else if(safeAction==2) {
			/
			if (momdp_speed_ < 0.6) {
				momdp_speed_ -= 0.2;
			}
			else if(momdp_speed_ < 1.0) {
				momdp_speed_-=0.3;
			} else {
			/
				momdp_speed_ -= 0.7;
		//		if(momdp_speed_<=0.1) momdp_speed_ = 0.1;
			
		}
        if(momdp_speed_<=0.0) momdp_speed_ = 0.0;
        if(momdp_speed_>=2.0) momdp_speed_ = 2.0;
		
		if(safeAction==2) brake_counts++;
		if(safeAction==1) brake_counts=0;

	//	if(brake_counts>=5)  momdp_speed_=0.1;
		*/
		

		cout<<"momdp_spped "<<momdp_speed_<<endl;


}


void ped_momdp::publishMarker(int id,vector<double> belief)
{
	visualization_msgs::MarkerArray markers;
	uint32_t shape = visualization_msgs::Marker::CUBE;
	//cout<<"belief vector size "<<belief.size()<<endl;
	for(int i=0;i<belief.size();i++)
	{
		visualization_msgs::Marker marker;			

		char buf[100];
		sprintf(buf,"%s%s",ModelParams::rosns,"/map");
		marker.header.frame_id=buf;
		marker.header.stamp=ros::Time::now();
		marker.ns="basic_shapes";
		marker.id=i;
		marker.type=shape;
		marker.action = visualization_msgs::Marker::ADD;

		double px=0,py=0;
		//px=RealWorldPt->ped_list[RealWorldPt->pedInView_list[id]].w;
		//py=RealWorldPt->ped_list[RealWorldPt->pedInView_list[id]].h;
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
	markers_pubs[id].publish(markers);

}
void ped_momdp::publishBelief()
{
	vector<vector<double> > ped_beliefs=RealSimulator->GetBeliefVector(solver->root_->particles());	
	//cout<<"belief vector size "<<ped_beliefs.size()<<endl;
	for(int i=0;i<ped_beliefs.size();i++)
	{
		publishMarker(i,ped_beliefs[i]);
	}

}
