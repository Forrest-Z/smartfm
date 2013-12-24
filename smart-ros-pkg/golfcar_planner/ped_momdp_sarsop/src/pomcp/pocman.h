#ifndef TIGER_H
#define TIGER_H

#include "simulator.h"
#include "coord.h"
#include "grid.h"

class TIGER_STATE : public STATE
{
public:
    int  tigerpos;
};
/*
class TIGER_BELIEF
{
public:
	BELIEF_STATE &beliefState;
	ROCKSAMPLE_BELIEF(BELIEF_STATE &b):beliefState(b)
	{}
	bool operator <(const TIGER_BELIEF &tb) const
	{

	}
}*/

class TIGER : public SIMULATOR
{
public:
    TIGER();
    //virtual void UpdateModel(int);
    virtual STATE* Copy(const STATE& state) const;
    //virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    void GeneratePreferred(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    virtual bool Step(STATE& state, int action,
        int& observation, double& reward) const;
    void DisplayBeliefs(const BELIEF_STATE&, std::ostream&)const;
	bool Init_QMDP();
	double GetReward(int,int);
	double TransFn(int,int,int);
	void DisplayQMDP();
	void UpdateBelief(int,int);
	int QMDP_SelectAction();
	void Reset() { prob_l=0.5;}
	VNODE* MapBelief(VNODE*vn,VNODE*r,const HISTORY & h,int);
	VNODE* Find(VNODE*v1,const HISTORY &h);
	bool Same(const BELIEF_STATE& beliefState1,const BELIEF_STATE& beliefState2,double & distance);
    //void DisplayState(const STATE&, std::ostream&) const;
    std::vector<VNODE*> vnode_list;
	std::vector<HISTORY> history_list;
protected:
    
    //ros::ServiceClient * client_pt;
	double prob_l;
    int Size;
    int trans[5][20][5][20];
    int walk_dirs[5][20][2];
    double qValue[2][3];
    double Value[2];
    enum
    {
        LISTEN,
        OPEN_LEFT,
        OPEN_RIGHT
    };
private:
    mutable MEMORY_POOL<TIGER_STATE> MemoryPool;
};

#endif
