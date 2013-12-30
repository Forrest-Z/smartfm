#ifndef PEDESTRIAN_DYNAMIC_UTOWN_H
#define PEDESTRIAN_DYNAMIC_UTOWN_H

#include "simulator.h"
#include "coord.h"
#include "grid.h"
#include <vector>
#include <utility>

class PEDESTRIAN_DYNAMIC_UTOWN_STATE:public STATE
{
public:
    COORD RobPos;
    COORD PedPos;
    int   Vel;
    int   Goal;
};

class PEDESTRIAN_DYNAMIC_UTOWN:public SIMULATOR
{
public:
	PEDESTRIAN_DYNAMIC_UTOWN(int x_size,int y_size);
	//virtual void UpdateModel(int);
	virtual STATE* Copy(const STATE& state) const;
	virtual void Validate(const STATE& state) const;
	virtual STATE* CreateStartState() const;
	virtual void FreeState(STATE* state) const;
	virtual bool Step(STATE& state, int action,
	OBS_TYPE& observation, double& reward) const;
	int GetNumOfStates();
	int MaxObs() const
	{return X_SIZE*Y_SIZE*(Y_SIZE+5)*3+2; }

	double Heuristic(STATE& state);
	void DisplayBeliefs(const BELIEF_STATE&, std::ostream&)const;
	void DisplayState(const STATE&, std::ostream&) const;
	VNODE* MapBelief(VNODE*vn,VNODE*r,const HISTORY &h,int);
	VNODE* Find(VNODE*v1,const HISTORY &h);
	VNODE* Find_old(VNODE*v1,const HISTORY &h);

	bool Same(const BELIEF_STATE& beliefState1,const BELIEF_STATE& beliefState2,double & distance);
	void PedStep(STATE& state) const;
	virtual void RobStep(STATE& state, int action) const;

	std::vector<VNODE*> vnode_old;
	std::vector<VNODE*> vnode_list;
	std::vector<HISTORY> history_list;
	int FindIndex(VNODE*v)
	{
		for(int i=0;i<vnode_list.size();i++)
		{
			if(vnode_list[i]==v) 	return i;
		}
		return -1;
	}
	int X_SIZE;
	int Y_SIZE;
	double model[10][20][6][10][20];  
	std::vector<std::pair<int,int> > rob_map;
    COORD currRobPos;
    COORD currPedPos;
    int   currVel;
	
protected:
    
    //ros::ServiceClient * client_pt;

    enum
    {
        ACT_CUR,
        ACT_ACC,
        ACT_DEC
    };
private:
    mutable MEMORY_POOL<PEDESTRIAN_DYNAMIC_UTOWN_STATE> MemoryPool;
};

#endif
