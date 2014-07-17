#include "controller.h"
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

Controller::Controller(ros::NodeHandle& nh, bool fixed_path, double pruning_constant, double pathplan_ahead):  worldStateTracker(worldModel), worldBeliefTracker(worldModel, worldStateTracker), fixed_path_(fixed_path), pathplan_ahead_(pathplan_ahead)
{
	cout << "fixed_path = " << fixed_path_ << endl;
	cout << "pathplan_ahead = " << pathplan_ahead_ << endl;
	Globals::config.pruning_constant = pruning_constant;

    nh.param("pruning_constant", Globals::config.pruning_constant, 0.0);

	global_frame_id = ModelParams::rosns + "/map";
	cerr <<"DEBUG: Entering Controller()"<<endl;
	control_freq=ModelParams::control_freq;

	cerr << "DEBUG: Initializing publishers..." << endl;
    believesPub_ = nh.advertise<ped_momdp_sarsop::peds_believes>("peds_believes",1);
    cmdPub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_pomdp",1);
	actionPub_ = nh.advertise<visualization_msgs::Marker>("pomdp_action",1);
	pathPub_= nh.advertise<nav_msgs::Path>("pomdp_path_repub",1, true);
	pathSub_= nh.subscribe("plan", 1, &Controller::RetrievePathCallBack, this);
	goal_pub=nh.advertise<visualization_msgs::MarkerArray> ("pomdp_goals",1);
	start_goal_pub=nh.advertise<ped_pathplan::StartGoal> ("ped_path_planner/planner/start_goal", 1);

	markers_pub=nh.advertise<visualization_msgs::MarkerArray>("pomdp_belief",1);
    pedStatePub_=nh.advertise<sensor_msgs::PointCloud>("ped_state", 1);
    pedPredictionPub_ = nh.advertise<sensor_msgs::PointCloud>("ped_prediction", 1);
	safeAction=2;
	target_speed_=0.0;
	goal_reached=false;
	cerr << "DEBUG: Init simulator" << endl;
	initSimulator();
    //RetrievePaths();

	cerr <<"DEBUG: before entering controlloop"<<endl;
    timer_ = nh.createTimer(ros::Duration(1.0/control_freq), &Controller::controlLoop, this);
	timer_speed=nh.createTimer(ros::Duration(0.05), &Controller::publishSpeed, this);

}



bool Controller::getObjectPose(string target_frame, tf::Stamped<tf::Pose>& in_pose, tf::Stamped<tf::Pose>& out_pose) const
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
void Controller::initSimulator()
{
  Globals::config.root_seed=1024;
  //Globals::config.n_belief_particles=2000;
  Globals::config.n_particles=300;
  Globals::config.time_per_move = (1.0/ModelParams::control_freq) * 0.9;
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



Controller::~Controller()
{
    geometry_msgs::Twist cmd;
    cmd.angular.z = 0;
    cmd.linear.x = 0;
    cmdPub_.publish(cmd);
}



void Controller::updateSteerAnglePublishSpeed(geometry_msgs::Twist speed)
{
    //cmdPub_.publish(speed);
}

void Controller::publishSpeed(const ros::TimerEvent &e)
{
	geometry_msgs::Twist cmd;
	cmd.angular.z = 0;       	
	cmd.linear.x = target_speed_;
	cout<<"publishing cmd speed "<<target_speed_<<endl;
	cmdPub_.publish(cmd);
}



void Controller::publishROSState()
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




void Controller::publishAction(int action)
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

		// Set the scale of the marker in meters
		marker.scale.x = 0.6;
		marker.scale.y = 3;
		marker.scale.z = 0.6;

		marker.color.r = marker_colors[action_map[action]][0];
        marker.color.g = marker_colors[action_map[action]][1];
		marker.color.b = marker_colors[action_map[action]][2];
		marker.color.a = 1.0;

		ros::Duration d(1/control_freq);
		marker.lifetime=d;
		actionPub_.publish(marker);
}

COORD poseToCoord(const tf::Stamped<tf::Pose>& pose) {
	COORD coord;
	coord.x=pose.getOrigin().getX();
	coord.y=pose.getOrigin().getY();
	return coord;
}


geometry_msgs::PoseStamped Controller::getPoseAhead(const tf::Stamped<tf::Pose>& carpose) {
    static int last_i = -1;
    static double last_yaw = 0;
    static COORD last_ahead;
	auto& path = worldModel.path;
	COORD coord = poseToCoord(carpose);

	int i = path.nearest(coord);
    COORD ahead;
    double yaw;

    if (i == last_i) {
        yaw = last_yaw;
        ahead = last_ahead;
    } else {
        int j = path.forward(i, pathplan_ahead_);
        //yaw = (path.getYaw(j) + path.getYaw(j-1)+ path.getYaw(j-2)) / 3;
        yaw = path.getYaw(j);
        ahead = path[j];
        last_yaw = yaw;
        last_i = i;
    }

	auto q = tf::createQuaternionFromYaw(yaw);
	tf::Pose p(q, tf::Vector3(ahead.x, ahead.y, 0));
	tf::Stamped<tf::Pose> pose_ahead(p, carpose.stamp_, carpose.frame_id_);
	geometry_msgs::PoseStamped posemsg;
	tf::poseStampedTFToMsg(pose_ahead, posemsg);
	return posemsg;
}

void Controller::sendPathPlanStart(const tf::Stamped<tf::Pose>& carpose) {
	if(fixed_path_ && worldModel.path.size()>0)  return;

	ped_pathplan::StartGoal startGoal; 
	geometry_msgs::PoseStamped pose;
	tf::poseStampedTFToMsg(carpose, pose);

	// set start
	if(pathplan_ahead_ > 0 && worldModel.path.size()>0) {
		startGoal.start = getPoseAhead(carpose);
	} else {
		startGoal.start=pose;
	}

	// set goal
	// create door
	//pose.pose.position.x=17;
	//pose.pose.position.y=52;

	// after CREATE door
	//pose.pose.position.x=19.5;
	//pose.pose.position.y=55.5;
	
	// before create door
	pose.pose.position.x=18.8;
	pose.pose.position.y=44.5;
	
	// large map goal
	//pose.pose.position.x=108;
	//pose.pose.position.y=143;
	
	startGoal.goal=pose;
	start_goal_pub.publish(startGoal);	
}

void Controller::RetrievePathCallBack(const nav_msgs::Path::ConstPtr path)  {
	cout<<"receive path from navfn "<<path->poses.size()<<endl;
	if(fixed_path_ && worldModel.path.size()>0) return;

	if(path->poses.size()==0) return;
	Path p;
	for(int i=0;i<path->poses.size();i++) {
        COORD coord;
		coord.x=path->poses[i].pose.position.x;
		coord.y=path->poses[i].pose.position.y;
        p.push_back(coord);
	}

	if(pathplan_ahead_>0 && worldModel.path.size()>0) {
        double pd = worldModel.path.mindist(p[0]);
        if(pd < 2 * ModelParams::PATH_STEP) {
            // only accept new path if the starting point is close to current path
            worldModel.path.cutjoin(p);
            auto pi = worldModel.path.interpolate();
            worldModel.setPath(pi);
        }
	} else {
		worldModel.setPath(p.interpolate());
	}

	publishPath(path->header.frame_id, worldModel.path);
}

void Controller::publishPath(const string& frame_id, const Path& path) {
	nav_msgs::Path navpath;
	ros::Time plan_time = ros::Time::now();

	navpath.header.frame_id = frame_id;
	navpath.header.stamp = plan_time;
	
	for(const auto& s: path) {
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = plan_time;
		pose.header.frame_id = frame_id;
		pose.pose.position.x = s.x;
		pose.pose.position.y = s.y;
		pose.pose.position.z = 0.0;
		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 0.0;
		pose.pose.orientation.w = 1.0;
		navpath.poses.push_back(pose);
	}

	pathPub_.publish(navpath);
}

void Controller::controlLoop(const ros::TimerEvent &e)
{
        static double starttime=get_time_second();
        cout<<"*********************"<<endl;
	    cout<<"entering control loop"<<endl;
        cout<<"current time "<<get_time_second()-starttime<<endl;
        tf::Stamped<tf::Pose> in_pose, out_pose;

  
        /****** update world state ******/
        ros::Rate err_retry_rate(10);

		// transpose to base link for path planing
		in_pose.setIdentity();
		in_pose.frame_id_ = ModelParams::rosns + "/base_link";
		if(!getObjectPose(global_frame_id, in_pose, out_pose)) {
			cerr<<"transform error within control loop"<<endl;
			cout<<"laser frame "<<in_pose.frame_id_<<endl;
            err_retry_rate.sleep();
            return;
		}

		sendPathPlanStart(out_pose);
		if(worldModel.path.size()==0) return;

		// transpose to laser frame for ped avoidance
		in_pose.setIdentity();
		in_pose.frame_id_ = ModelParams::rosns + ModelParams::laser_frame;
		if(!getObjectPose(global_frame_id, in_pose, out_pose)) {
			cerr<<"transform error within control loop"<<endl;
			cout<<"laser frame "<<in_pose.frame_id_<<endl;
            err_retry_rate.sleep();
            return;
		}

        worldStateTracker.updateVel(real_speed_);

		COORD coord = poseToCoord(out_pose);
		cout << "transformed pose = " << coord.x << " " << coord.y << endl;
		worldStateTracker.updateCar(coord);

        worldStateTracker.cleanPed();

		PomdpState curr_state = worldStateTracker.getPomdpState();
		publishROSState();

        cout << "root state:" << endl;
		despot->PrintState(curr_state, cout);

        /****** check world state *****/

		if(worldModel.isGlobalGoal(curr_state.car)) {
			goal_reached=true;
		}
		if(goal_reached==true) {
			safeAction=2;

			target_speed_=real_speed_;
		    target_speed_ -= 0.5;
			if(target_speed_<=0.0) target_speed_ = 0.0;

            // shutdown the node after reaching goal
            // TODO consider do this in simulaiton only for safety
            if(real_speed_ <= 1e-5) {
                ros::shutdown();
            }
			return;
		}

		if(worldStateTracker.emergency())
		{
			target_speed_=-1;
			return;
		}


        /****** update belief ******/

		cout<<"before belief update"<<endl;
		worldBeliefTracker.update();
		cout<<"after belief update"<<endl;

        publishPedsPrediciton();

		vector<PomdpState> samples = worldBeliefTracker.sample(1000);
		vector<State*> particles = despot->ConstructParticles(samples);

        /*
		double sum=0;
		for(int i=0; i<particles.size(); i++)
			sum+=particles[i]->weight;
		cout<<"particle weight sum "<<sum<<endl;
        */

		ParticleBelief *pb=new ParticleBelief(particles, despot);
        despot->PrintState(*(pb->particles()[0]));
		solver->belief(pb);

        /****** random simulation for verification purpose ******/
        /*
        Random rand((unsigned)1012312);
        cout << "Starting simulation" << endl;
        for (int i = 0; i < 100; i ++) {
            uint64_t obs;
            double reward;

            PomdpState state = worldBeliefTracker.sample();
            if (state.num <= 1)
                continue;

            cout << "Initial state: " << endl;
            despot->PrintState(state);

	        sensor_msgs::PointCloud pc;
        	pc.header.frame_id="/map";
            pc.header.stamp=ros::Time::now();
            ros::Rate loop_rate(ModelParams::control_freq);
            for (int j = 0; j < 20; j ++) {

                cout << "Step " << j << endl;
                int action = rand.NextInt(despot->NumActions());
                double r = rand.NextDouble();
                despot->Step(state, r, action, reward, obs);
                cout << "Action = " << action << endl;
                despot->PrintState(state);
                cout << "Reward = " << reward << endl;
                cout << "Obs = " << obs << endl;
                worldBeliefTracker.printBelief();
                for(int k=0;k<state.num;k++) {
       		        geometry_msgs::Point32 p;
                    p.x=state.peds[k].pos.x;
                    p.y=state.peds[k].pos.y;
                    p.z=1.0;
                    pc.points.push_back(p);
                }

                pedStatePub_.publish(pc); 
  //              publishBelief();
                loop_rate.sleep();
            }

        }
        cout << "End of simulation" << endl;
        */


        /****** solve for safe action ******/
		safeAction=solver->Search();

		//actionPub_.publish(action);
		publishAction(safeAction);

		cout<<"safe action = "<<safeAction<<endl;


		publishBelief();


        /****** update target speed ******/

		target_speed_=real_speed_;
        if(safeAction==0) {}
		else if(safeAction==1) target_speed_ += 0.15*2;
		else if(safeAction==2) target_speed_ -= 0.25*2;
		if(target_speed_<=0.0) target_speed_ = 0.0;
		if(target_speed_>=ModelParams::VEL_MAX) target_speed_ = ModelParams::VEL_MAX;

		cout<<"target_speed = "<<target_speed_<<endl;
		delete pb;
}

void Controller::publishPedsPrediciton() {
    vector<PedStruct> peds = worldBeliefTracker.predictPeds();
    sensor_msgs::PointCloud pc;
    pc.header.frame_id=global_frame_id;
    pc.header.stamp=ros::Time::now();
    for(const auto& ped: peds) {
        geometry_msgs::Point32 p;
        p.x = ped.pos.x;
        p.y = ped.pos.y;
        p.z = 1.0;
        pc.points.push_back(p);
    }
    pedPredictionPub_.publish(pc);
}


void Controller::publishMarker(int id,PedBelief & ped)
{
	//cout<<"belief vector size "<<belief.size()<<endl;
	std::vector<double> belief = ped.prob_goals;
	uint32_t shape = visualization_msgs::Marker::CUBE;
    uint32_t shape_text=visualization_msgs::Marker::TEXT_VIEW_FACING;
	for(int i=0;i<belief.size();i++)
	{
		visualization_msgs::Marker marker;
		visualization_msgs::Marker marker_text;

		marker.header.frame_id=global_frame_id;
		marker.header.stamp=ros::Time::now();
		marker.ns="basic_shapes";
		marker.id=id*ped.prob_goals.size()+i;
		marker.type=shape;
		marker.action = visualization_msgs::Marker::ADD;

		marker_text.header.frame_id=global_frame_id;
		marker_text.header.stamp=ros::Time::now();
		marker_text.ns="basic_shapes";
		marker_text.id=id*ped.prob_goals.size()+i+1000;
		marker_text.type=shape_text;
		marker_text.action = visualization_msgs::Marker::ADD;


		double px=0,py=0;
		px=ped.pos.x;
		py=ped.pos.y;
		marker_text.pose.position.x = px;
		marker_text.pose.position.y = py;
		marker_text.pose.position.z = 0.5;
		marker_text.pose.orientation.x = 0.0;
		marker_text.pose.orientation.y = 0.0;
		marker_text.pose.orientation.z = 0.0;
		marker_text.pose.orientation.w = 1.0;
	//	cout<<"belief entries "<<px<<" "<<py<<endl;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		//if(marker.scale.y<0.2) marker.scale.y=0.2;
		marker_text.scale.z = 1.0;
		//
		// Set the color -- be sure to set alpha to something non-zero!
		//marker.color.r = 0.0f;
		//marker.color.g = 1.0f;
		//marker.color.b = 0.0f;
		//marker.color.a = 1.0;
		marker_text.color.r = 0.0;
        marker_text.color.g = 0.0;
		marker_text.color.b = 0.0;//marker.lifetime = ros::Duration();
		marker_text.color.a = 1.0;
        marker_text.text = to_string(ped.id); 

		ros::Duration d1(1/control_freq);
		marker_text.lifetime=d1;


		px=0,py=0;
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

		ros::Duration d2(1/control_freq);
		marker.lifetime=d2;
		markers.markers.push_back(marker);
        //markers.markers.push_back(marker_text);
	}
}
void Controller::publishBelief()
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
