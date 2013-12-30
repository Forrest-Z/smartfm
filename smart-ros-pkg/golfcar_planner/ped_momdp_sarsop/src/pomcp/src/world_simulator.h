#ifndef WORLD_SIMULATOR_H
#define WORLD_SIMULATOR_H
#include "pedestrian_state.h"
#include "SFM.h"
#include "map.h"
#include "window.h"
#include "param.h"
#include "math_utils.h"

//here need to modify the obs type according to different simulator
typedef long long OBS_T;
class WorldSimulator{
public:
	WorldSimulator(double random_num=0.62)
		:window(&world_map,0),car(&world_map),sfm(&world_map,&window),unif(random_num)
    {
		if(ModelParams::debug)
		{
			cout<<"seed "<<random_num<<endl;
			cout<<"num ped in view "<<" "<<NumPedInView()<<endl;
			cout<<"rob map size "<<window.rob_map.size()<<endl;
		}
    }

	void Init()
	{
		t_stamp=0;
		car.carPos=robPos=windowOrigin=0;	
		car.w=world_map.global_plan[robPos][0];
		car.h=world_map.global_plan[robPos][1];
		velGlobal=1.0;


		if(ModelParams::FixedPath)     InitTestPaths();   //This is for init user specific path for the pedestrians
		//InitPedestriansBlank();

		ShiftWindow();
		InitPedestriansBlank();
		ShiftWindow();
		//sfm.UpdateSFM();
	}
	void InitTestPaths()
	{
		ped_paths[0][0][0]=180;
		ped_paths[0][0][1]=130;

		for(int i=1;i<10;i++)
		{
			ped_paths[0][i][0]=180-i*20;
			ped_paths[0][i][1]=130;
		}
		ped_paths[0][10][0]=-1000;
		ped_paths[0][10][1]=-1000;
	}

	Pedestrian GetPedPose(int i)
	{
		Pedestrian ped;
		window.GlobalToLocal(ped_list[pedInView_list[i]].w,ped_list[pedInView_list[i]].h,ped.w,ped.h);
		ped.id=pedInView_list[i];
		return ped;
	}
	Car GetCarPos()
	{
		Car local_car;
		int car_w=world_map.global_plan[robPos][0];
		int car_h=world_map.global_plan[robPos][1];
		window.GlobalToLocal(car_w,car_h,local_car.w,local_car.h);
		if(ModelParams::debug) 	cout<<"local car "<<local_car.w<<" "<<local_car.h<<endl;
		return local_car;
	}
	void SetSeed(unsigned new_seed_)
	{
		unif.seed_=new_seed_;
	}
	OBS_T CalCurrObs()
	{
		int X_SIZE=ModelParams::XSIZE;
		int Y_SIZE=ModelParams::YSIZE;
		GetCurrState();
		OBS_T obs=0;// = state.Vel*(X_SIZE*Y_SIZE*rob_map.size())+state.RobPos.Y*(X_SIZE*Y_SIZE)+state.PedPos.X*Y_SIZE+state.PedPos.Y;
		OBS_T robObs=curr_state.Vel+curr_state.RobPos.Y*3;
		OBS_T robObsMax=3*ModelParams::RMMax;  //max length of the rob_map
		OBS_T pedObsMax=X_SIZE*Y_SIZE;
		OBS_T pedObs=0;
		//for(int i=0;i<state.PedPoses.size();i++)
		for(int i=0;i<curr_state.num;i++)
		{
			OBS_T this_obs=curr_state.PedPoses[i].first.X*Y_SIZE+curr_state.PedPoses[i].first.Y;	
			pedObs=pedObs*pedObsMax+this_obs;
		}
		obs=pedObs*robObsMax+robObs;
		if(ModelParams::debug) 	cout<<"world observation "<<obs<<endl;
		return obs;
	}
	OBS_T GetCurrObs()
	{return curr_obs;}

	bool InSafeZone(int w,int h)
	{
		int car_w=world_map.global_plan[robPos][0];
		int car_h=world_map.global_plan[robPos][1];
		if(fabs(w-car_w)+fabs(h-car_h)<=ModelParams::rln*3) return false;	
		else return true;
	}
	Pedestrian InitOnePed()
	{
		/*	
		int goal=unif.next()*ModelParams::NGOAL;
		bool ok=false;
		int rob_w,rob_h,x,y;
		window.GlobalToLocal(world_map.global_plan[robPos][0],world_map.global_plan[robPos][1],rob_w,rob_h);
		while(!ok)
		{
			x=unif.next()*ModelParams::XSIZE;
			y=unif.next()*ModelParams::YSIZE;
			if(fabs(x-rob_w)+fabs(y-rob_h)>3) ok=true;
		}
		int ped_w,ped_h;
		window.LocalToGlobal(x,y,ped_w,ped_h);
		cout<<"initial ped x y goal "<<ped_w<<" "<<ped_h<<" "<<goal<<endl;
		return Pedestrian(ped_w,ped_h,goal,0);*/
			

		int goal=unif.next()*ModelParams::NGOAL;
		int goal_w,goal_h;
		goal_w=world_map.goal_pos[goal][0];
		goal_h=world_map.goal_pos[goal][1];
		int rln=ModelParams::rln/2;
		int range=(ModelParams::GOAL_DIST-2)*ModelParams::rln;
		int sum=0;
		for(int x=goal_w-range;x<goal_w+range;x+=rln)
			for(int y=goal_h-range;y<goal_h+range;y+=rln)
			{
				if(world_map.InMap(x,y)&&InSafeZone(x,y))  sum++; 
			}
		int grid=unif.next()*sum;
		sum=0;
		bool inner=false;
		for(int x=goal_w-range;x<goal_w+range&&inner==false;x+=rln)
			for(int y=goal_h-range;y<goal_h+range;y+=rln)
			{
				if(world_map.InMap(x,y)&&InSafeZone(x,y))  sum++; 
				if(sum>grid)  
				{
					inner=true;
					//ped_list.push_back(Pedestrian(x,y,goal,0));
					if(ModelParams::debug)  cout<<"initial ped "<<" "<<x<<" "<<y<<" "<<goal<<endl;
					return Pedestrian(x,y,goal,0);
					break;
				}
			}

	}
	void InitPedestriansTest()
	{
		ped_list.clear();
		pedInView_list.clear();
		for(int i=0;i<NumPedTotal;i++)
		{
			ped_list.push_back(InitOnePed());
		}
	}
	void InitPedestriansBlank()
	{
		ped_list.clear();
		pedInView_list.clear();
		for(int i=0;i<NumPedTotal;i++)
		{
			if(ModelParams::FixedPath)
			{
				ped_list.push_back(Pedestrian(ped_paths[0][i][0],ped_paths[0][i][1],2,0));
				cout<<ped_list[i].w<<" "<<ped_list[i].h<<endl;
			}
			else 	ped_list.push_back(InitOnePed());
			if(ModelParams::FixedPath)
			{
				ped_list[i].ts=i;
			}
		}
		//if(ModelParams::FixedPath) ChangePath();
		//Pedestrian ped1(20*ModelParams::rln,8*ModelParams::rln,2,0);
		//ped_list.push_back(ped1);
		//Pedestrian ped2(18*ModelParams::rln,7*ModelParams::rln,3,0);
		//ped_list.push_back(ped2);
		//Pedestrian ped3(17*ModelParams::rln,5*ModelParams::rln,0,0);	
		//ped_list.push_back(ped3);
		//cout<<"ped 2 "<<ped2.w<<" "<<ped2.h<<endl;
		//char buf[50];
		//sprintf(buf,"/ped_pose_%d",0);
		//	ped_pubs.push_back(n_pt->advertise<geometry_msgs::PoseStamped>(buf,1000));

	}

	bool OneStep(int action)
	{
		t_stamp++;
		if(GoalReached())       return true;	
		if(InCollision(action)) return true;  	
		UpdateCar(action);
		Display();
		UpdatePed();
		
		if(ModelParams::FixedPath)
		{
			for(int i=0;i<ped_list.size();i++)
				ped_list[i].ts++;
		}
			
		if(ModelParams::FixedPath)  ChangePath();
		GetCurrState();
			
		if(ModelParams::debug)
		{
			cout<<"before shift window"<<endl;
			cout << "Rob Pos: " << window.rob_map[curr_state.RobPos.Y].first << " " <<window.rob_map[curr_state.RobPos.Y].second <<endl;
			//for(int i=0;i<state.PedPoses.size();i++)
			for(int i=0;i<curr_state.num;i++)
			{
				cout << "Ped Pos: " << curr_state.PedPoses[i].first.X << " " <<curr_state.PedPoses[i].first.Y <<endl;
				cout << "Goal: " << curr_state.PedPoses[i].second << endl;
			}
			cout << "Vel: " << curr_state.Vel << endl;
			Display();
		}
		
		curr_obs=CalCurrObs();

		//ShiftWindow();
		return false;
	}

	void ChangePath()
	{
		for(int i=0;i<ped_list.size();i++)
		{
			ped_list[i].w=ped_paths[0][ped_list[i].ts][0];
			ped_list[i].h=ped_paths[0][ped_list[i].ts][1];
		}
	}

	void ShiftWindow()
	{
		if(robPos-windowOrigin>=ModelParams::path_rln*5) 	  windowOrigin=robPos;
		window.RollWindow(windowOrigin);
		pedInView_list.clear();
		for(int i=0;i<ped_list.size();i++)
		{
			if(ped_list[i].w==-1000&&ped_list[i].h==-1000)
			{
				ped_list[i]=InitOnePed();
			}
			if(window.InWindow(ped_list[i].w,ped_list[i].h)&&(pedInView_list.size()<ModelParams::N_PED_IN))
			{
				pedInView_list.push_back(i);	
			}
		}
		cout<<"Num Ped In View = "<<pedInView_list.size()<<endl;
		sfm.UpdateSFM();//update the precompute staff
		if(ModelParams::debug)
		{
			cout<<"local obs map"<<endl;
			for(int i=0;i<ModelParams::XSIZE;i++)
			{
				for(int j=0;j<ModelParams::YSIZE;j++)
				{
					int w,h;
					window.LocalToGlobal(i,j,w,h);	
					//cout<<w<<" "<<h<<" ";
					cout<<world_map.Free(w,h)<<" ";
				}
				cout<<endl;
			}
		}

	}
	void ShiftWindow(int pos)
	{
		windowOrigin=pos;
		window.RollWindow(windowOrigin);
		pedInView_list.clear();
		for(int i=0;i<ped_list.size();i++)
		{
			if(ped_list[i].w==-1000&&ped_list[i].h==-1000)
			{
				ped_list[i]=InitOnePed();
			}
			if(window.InWindow(ped_list[i].w,ped_list[i].h)&&(pedInView_list.size()<ModelParams::N_PED_IN))
			{
				pedInView_list.push_back(i);	
			}
		}
		sfm.UpdateSFM();//update the precompute staff
	}
	void UpdateCarGood(int action)
	{
		int &robY=robPos;
		int rob_vel=velGlobal;

		double vel_p=unif.next();
		if(rob_vel==0)
		{
			if(vel_p<0.9)   
			{
			}
			else
			{
			//	robY+=ModelParams::path_rln*0.8;
			}

		}
		else if(rob_vel==1)
		{
			if(vel_p<0.8) robY+=ModelParams::path_rln;
			else if(vel_p<0.9) robY+=2*ModelParams::path_rln;
		}
		else
		{
			if(vel_p<0.8) robY+=2*ModelParams::path_rln;
			else if(vel_p<0.9) robY+=ModelParams::path_rln;
		}

		//if(robY>rob_map.size()-1) robY=rob_map.size()-1;
		//TODO: terminal condition
		double act_p=unif.next();
		if(action==1) rob_vel++;
		if(action==2) 
		{
			if(act_p<0.5) rob_vel--;
			else 		  rob_vel-=2; 
		}
		
		if(rob_vel<0) rob_vel=0;
		if(rob_vel>2) rob_vel=2;	
		velGlobal=rob_vel;
	}

	void UpdateCar(int action)
	{
		if(ModelParams::goodrob==1) {
			UpdateCarGood(action);
			return;
		}
		int &robY=robPos;
		int rob_vel=velGlobal;

		double vel_p=unif.next();
		if(rob_vel==0)
		{
			if(vel_p<0.9)   
			{
			}
			else
			{
				robY+=ModelParams::path_rln;
			}

		}
		else if(rob_vel==1)
		{
			if(vel_p<0.8) robY+=ModelParams::path_rln;
			else if(vel_p<0.9) robY+=2*ModelParams::path_rln;
		}
		else
		{
			if(vel_p<0.8) robY+=2*ModelParams::path_rln;
			else if(vel_p<0.9) robY+=ModelParams::path_rln;
		}

		//if(robY>rob_map.size()-1) robY=rob_map.size()-1;
		//TODO: terminal condition
		double act_p=unif.next();

		if(action==1)
		{
			if(rob_vel==0)
			{
				if(act_p<0.7) rob_vel++;
				else if(act_p<0.8) rob_vel+=2;
				else rob_vel=0;
			}
			else if(rob_vel==1)
			{
				if(act_p<0.8) rob_vel=2;
				else if(act_p<0.9) rob_vel=1;
				else rob_vel=0;
			}
			else if(rob_vel==2)
			{
				if(act_p<0.8) rob_vel=2;
				else if(act_p<0.9) rob_vel=1;
				else rob_vel=0;
			}
		}
		else if(action==2)
		{
			if(rob_vel==0)
			{
				if(act_p<0.9) {}
				else {rob_vel++;}
			}
			else if(rob_vel==1)
			{
				if(act_p<0.9){rob_vel--;}
			}
			else if(rob_vel==2)
			{
				if(act_p<0.7){rob_vel--;}
				else if(act_p<0.8) {}
				else {rob_vel-=2;}
			}	
		}
		else
		{
			if(rob_vel==0)
			{
				if(act_p<0.9) rob_vel=0;
				else rob_vel=1;
			}
			else if(rob_vel==1)
			{
				if(act_p<0.7) rob_vel=1;
				else if(act_p<0.8) rob_vel=2;
				else rob_vel=0;
			}
			else if(rob_vel==2)
			{
				if(act_p<0.8) rob_vel=2;
				else if(act_p<0.9) rob_vel=1;
				else 	rob_vel=0;
			}
		}

			
		
		if(rob_vel<0) rob_vel=0;
		if(rob_vel>2) rob_vel=2;	
		velGlobal=rob_vel;

	}

	void UpdatePed()
	{
		//here we update the ped poses based on the old car position
		sfm.debug=true;
		sfm.WorldTrans(ped_list,car,unif);	
		sfm.debug=false;

		
		car.w=world_map.global_plan[robPos][0];
		car.h=world_map.global_plan[robPos][1];
		car.carPos=robPos;
	}

	void UpdatePedPoseReal(Pedestrian ped)
	{
		int i;
		for( i=0;i<ped_list.size();i++)
		{
			if(ped_list[i].id==ped.id)
			{
				//found the corresponding ped,update the pose
				ped_list[i].w=ped.w;
				ped_list[i].h=ped.h;
				break;
			}
		}
		if(i==ped_list.size())   //not found, new ped
		{
			ped_list.push_back(ped);
		}
	}

	void UpdateRobPoseReal(Car world_car)
	{
		//need to find the closest point in the pre-defined path
		int next=robPos-1;
		int next_diff=10000;
		for(int i=robPos;i<world_map.pathLength&&i<robPos+ModelParams::rln;i++)
		{
			int curr_diff=abs(world_map.global_plan[i][0]-world_car.w)+abs(world_map.global_plan[i][1]-world_car.h);
			if(curr_diff<next_diff)
			{
				next_diff=curr_diff;	
				next=i;
			}
		}
		robPos=next;
		car.w=world_map.global_plan[robPos][0];
		car.h=world_map.global_plan[robPos][1];
	}
	void UpdateVelReal(double vel)
	{
		if(vel>2) vel=2;	
		if(vel<0) vel=0;
		velGlobal=vel;
	}


	void Display()
	{
		GetCurrState();
		cout<<"Car Pos "<<world_map.global_plan[robPos][0]<<" "<<world_map.global_plan[robPos][1]<<endl;
		cout<<"Window Pos "<<windowOrigin<<endl;
		cout<<"Rob Pos "<<robPos<<endl;
		cout<<"ped num "<<pedInView_list.size()<<endl;
		/*
		for(int i=0;i<pedInView_list.size();i++)
		{
			cout<<"Real Ped Pos "<<ped_list[pedInView_list[i]].w<<" "<<ped_list[pedInView_list[i]].h<<endl;
		}*/
		for(int i=0;i<ped_list.size();i++)
		{
			cout<<"Real Ped Pos "<<ped_list[i].w<<" "<<ped_list[i].h<<endl;
		}
	}
	PedestrianState GetCurrState()
	{
		int x,y;
		window.GlobalToLocal(world_map.global_plan[robPos][0],world_map.global_plan[robPos][1],x,y);
		if(ModelParams::debug) cout<<"rob pos global "<<world_map.global_plan[robPos][0]<<" "<<world_map.global_plan[robPos][1]<<endl;
		int i;
		for(i=0;i<window.rob_map.size();i++)
		{
			if(x==window.rob_map[i].first&&y==window.rob_map[i].second) 
			{
				curr_state.RobPos.Y=i;
				break;
			}
		}
		if(i==window.rob_map.size())  cout<<"!!!!!!!!car drift out of the predefined path"<<endl;
		for(int i=0;i<pedInView_list.size();i++)
		{
			int ped_w,ped_h;
			window.GlobalToLocal(ped_list[pedInView_list[i]].w,ped_list[pedInView_list[i]].h,ped_w,ped_h);
			if(ped_w<0) ped_w=0;
			if(ped_w>ModelParams::XSIZE-1) ped_w=ModelParams::XSIZE-1;
			if(ped_h<0) ped_h=0;
			if(ped_h>ModelParams::YSIZE-1) ped_h=ModelParams::YSIZE-1;
			curr_state.PedPoses[i].first.X=ped_w;
		    curr_state.PedPoses[i].first.Y=ped_h;	
			curr_state.PedPoses[i].second=ped_list[pedInView_list[i]].goal;
			curr_state.PedPoses[i].third=pedInView_list[i];
		}
		curr_state.num=pedInView_list.size();
		curr_state.Vel=velGlobal;
		return curr_state;
	}
	
	bool InCollision(int action)
	{
		GetCurrState();
		for(int i=0;i<pedInView_list.size();i++)
		{

			int pedX=curr_state.PedPoses[i].first.X;
			int pedY=curr_state.PedPoses[i].first.Y;
			int robY=curr_state.RobPos.Y;
			int rob_vel=curr_state.Vel;


			if(pedX==window.rob_map[robY].first&&window.rob_map[robY].second==pedY)
			{
				return true;
			}
			if(pedX==window.rob_map[robY].first&&pedY==window.rob_map[robY].second+1)
			{
				if(rob_vel==0&&action==1)  {return true;}
				if(rob_vel==1&&action<2)   {return true;}
				if(rob_vel==2)     			{return true;}
			}
			if(pedX==window.rob_map[robY].first&&pedY==window.rob_map[robY].second+2)
			{
				if(rob_vel==1&&action==1)  {return true;}
				if(rob_vel==2&&action<2)  {return true;}
			}

		}
		return false;
	}

	bool InRealCollision(int action)
	{
		GetCurrState();
		for(int i=0;i<pedInView_list.size();i++)
		{

			int pedX=curr_state.PedPoses[i].first.X;
			int pedY=curr_state.PedPoses[i].first.Y;
			int robY=curr_state.RobPos.Y;
			int rob_vel=curr_state.Vel;


			if(pedX==window.rob_map[robY].first&&window.rob_map[robY].second==pedY)
			{
				return true;
			}
		}
		return false;
	}
	bool GoalReached()
	{
		if(world_map.pathLength-robPos<ModelParams::path_rln*2.1)	 return true;
		//if(robPos>ModelParams::path_rln*(ModelParams::YSIZE-2))	 return true;
		else return false;
	}

	int NumPedInView()
	{
		return pedInView_list.size();
	}


	MyMap  world_map;
	MyWindow  window;
	Car car;
	SFM sfm;
	int NumPedTotal;
	int robPos;
	int windowOrigin;
	int t_stamp;
	double velGlobal;
	PedestrianState ped_state;
	vector<Pedestrian> ped_list;
	vector<int> pedInView_list;
	PedestrianState curr_state;
	OBS_T curr_obs;
	UtilUniform unif;
	int ped_paths[3][100][2];
};
#endif