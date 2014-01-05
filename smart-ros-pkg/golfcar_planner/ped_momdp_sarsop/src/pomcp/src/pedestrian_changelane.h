#ifndef PEDESTRIAN_CHANGELANE_H
#define PEDESTRIAN_CHANGELANE_H

#include "simulator.h"

#include "grid.h"
#include <vector>
#include <utility>
#include <cmath>
#include "SFM.h"
#include "pedestrian_state.h"


using namespace std;
class SFM;


class PEDESTRIAN_CHANGELANE:public SIMULATOR
{
public:
	PEDESTRIAN_CHANGELANE(int x_size,int y_size);
	//virtual void UpdateModel(int);
	virtual STATE* Copy(const STATE& state) const;
	virtual void Validate(const STATE& state) const;
	virtual STATE* CreateStartState() const;
	virtual void FreeState(STATE* state) const;
	virtual bool Step(STATE& state, int action,
	OBS_TYPE& observation, double& reward) const;
	int GetNumOfStates();
	OBS_TYPE MaxObs() const
	{
		OBS_TYPE ROB=(ModelParams::RMMax)*3;
		OBS_TYPE PED=pow(double(X_SIZE*Y_SIZE),double(ModelParams::N_PED_IN));
		return ROB*PED+2;
	}

	double Heuristic(STATE& state);
	void DisplayBeliefs(const BELIEF_STATE&, std::ostream&)const;
	vector<vector<double> > GetBeliefVector(const BELIEF_STATE& beliefs) const;
	void DisplayState(const STATE&, std::ostream&) const;
	VNODE* MapBelief(VNODE*vn,VNODE*r,const HISTORY &h,int);
	VNODE* Find(VNODE*v1,const HISTORY &h);
	VNODE* Find_old(VNODE*v1,const HISTORY &h);
	OBS_TYPE Observe(const PedestrianState state)const;
	bool Same(const BELIEF_STATE& beliefState1,const BELIEF_STATE& beliefState2,double & distance);
	void PedStep(PedestrianState&) const;
	virtual void RobStep(PedestrianState&, int ) const;
	virtual void ModifyObsStates(BELIEF_STATE &beliefs,STATE*state)const;
	void SetStartState(PedestrianState state);

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
	int lookup(const double probs[], double prob) const {
		int pos = 0;
		double sum = probs[0];
		while(sum < prob) {
			pos ++;
			sum += probs[pos];
		}
		return pos;
	}
	int X_SIZE;
	int Y_SIZE;
	double model[10][20][6][10][20];  
	std::vector<std::pair<int,int> > rob_map;
    COORD currRobPos;
    COORD currPedPos;
    int   currVel;
    
    PedestrianState*PedestrianState1;
    PedestrianState*PedestrianState2;
    int dir[8][2];
	SFM*sfm;

	PedestrianState startState;
	bool debug;

	double robotNoisyMove[3][3]; /*vel,move*/
	double robotMoveProbs[3][3]; 
	double robotVelUpdate[3][3][3]; /*action,vel,new vel*/ 
	double robotUpdateProb[3][3][3];

protected:
    
    //ros::ServiceClient * client_pt;

    enum
    {
        ACT_CUR,
        ACT_ACC,
        ACT_DEC
    };
private:
    mutable MEMORY_POOL<PedestrianState> MemoryPool;
};

#endif
