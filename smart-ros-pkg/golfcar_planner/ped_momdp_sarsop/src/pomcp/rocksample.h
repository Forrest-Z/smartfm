#ifndef ROCKSAMPLE_H
#define ROCKSAMPLE_H

#include "simulator.h"
#include "coord.h"
#include "grid.h"
#include <map>

class ROCKSAMPLE_STATE : public STATE
{
public:

    COORD AgentPos;
    struct ENTRY
    {
        bool Valuable;
        bool Collected;
        int Count;    				// Smart knowledge
        int Measured; 				// Smart knowledge
        double LikelihoodValuable;	// Smart knowledge
        double LikelihoodWorthless;	// Smart knowledge
        double ProbValuable;		// Smart knowledge
    };
    std::vector<ENTRY> Rocks;
    int Target; // Smart knowledge
};


class ROCKSAMPLE_BELIEF
{
public:
	int NumRocks;
	BELIEF_STATE &beliefState;
	ROCKSAMPLE_BELIEF(BELIEF_STATE &b,int nr):beliefState(b),NumRocks(nr)
	{}
	bool operator <(const ROCKSAMPLE_BELIEF &rb) const
	{
		BELIEF_STATE & beliefState2=rb.beliefState;
		const ROCKSAMPLE_STATE*rockstate1=safe_cast<const ROCKSAMPLE_STATE*>(beliefState.GetSample(0));
		const ROCKSAMPLE_STATE*rockstate2=safe_cast<const ROCKSAMPLE_STATE*>(beliefState2.GetSample(0));
		int pos1=rockstate1->AgentPos.X*7+rockstate1->AgentPos.Y;
		int pos2=rockstate2->AgentPos.X*7+rockstate2->AgentPos.Y;
		if(pos1<pos2) 	return true;
		if(pos1>pos2) 	return false;
	 	ROCKSAMPLE_STATE rockstate; 
		int rock_count1[20][2]={0};
		int rock_count2[20][2]={0};
		for (int i=0;i<beliefState.GetNumSamples();i++)
		{	
				const STATE* p_rockstate=beliefState.GetSample(i);
				rockstate=safe_cast<const ROCKSAMPLE_STATE&>(*p_rockstate);
				for(int j=0;j<NumRocks;j++)
				{
						if(rockstate.Rocks[j].Valuable==false ||rockstate.Rocks[j].Collected==true)
						{
								rock_count1[j][0]++;
						}
						else
						{
								rock_count1[j][1]++;
						}

				}
		}
		for (int i=0;i<beliefState2.GetNumSamples();i++)
		{
				const STATE* p_rockstate=beliefState.GetSample(i);
				rockstate=safe_cast<const ROCKSAMPLE_STATE&>(*p_rockstate);
				for(int j=0;j<NumRocks;j++)
				{
						if(rockstate.Rocks[j].Valuable==false ||rockstate.Rocks[j].Collected==true)
						{
								rock_count2[j][0]++;
						}
						else
						{
								rock_count2[j][1]++;
						}

				}
		}
		for(int i=0;i<NumRocks;i++)
		{
			double p1=(rock_count1[i][0]+0.0)/(rock_count1[i][0]+rock_count1[i][1]);
			double p2=(rock_count2[i][0]+0.0)/(rock_count2[i][0]+rock_count2[i][1]);
			if(p1<p2-0.1)   return true;
			if(p1>p2+0.1) 	return false;
		}
		return true;
	}
};

class ROCKSAMPLE : public SIMULATOR
{
public:

    ROCKSAMPLE(int size, int rocks);

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action,
        int& observation, double& reward) const;

    void GenerateLegal(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    void GeneratePreferred(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObservation, const STATUS& status) const;

    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState,
        std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;

	double QMDP_Heuristic(STATE*state);
	int map_time; 
	int map_counter;
	int QMDP_SelectAction(STATE*state);
	int  GetNumOfStates();
	void Init_QMDP();
	void Init_QMDP2();
	double QMDP_Value(ROCKSAMPLE_STATE*rockstate);
	int StateToN(STATE* state);
	void NToState(int n,ROCKSAMPLE_STATE*rockstate);
	void DisplayQMDP();
	void WriteQMDP();
	void LoadQMDP();
	virtual int GetSize()const{return Size;}
	virtual int GetNRocks()const{return NumRocks;}
	VNODE* MapBelief(VNODE*vn,VNODE*r,const HISTORY &h,int);
	VNODE* Find(VNODE*v1,const HISTORY &);
	int FindIndex(VNODE*v)
	{
		for(int i=0;i<vnode_list.size();i++)
		{
			if(vnode_list[i]==v) 	return i;
		}
		return -1;
	}
	VNODE* FindByIndex(int n)
	{
		return vnode_list[n];	
	}
	bool Same(const BELIEF_STATE& beliefState1,const BELIEF_STATE& beliefState2,double &);
	std::vector<VNODE*> vnode_list;
	std::vector<HISTORY> history_list;
protected:

    enum
    {
        E_NONE,
        E_GOOD,
        E_BAD
    };

    enum
    {
        E_SAMPLE = 4
    };

    void InitGeneral();
	void Init_10_4();
	void Init_4_4();
    void Init_7_8();
    void Init_11_11();
    int GetObservation(const ROCKSAMPLE_STATE& rockstate, int rock) const;
    int SelectTarget(const ROCKSAMPLE_STATE& rockstate) const;

    GRID<int> Grid;
    std::vector<COORD> RockPos;
    int Size, NumRocks;
    COORD StartPos;
    double HalfEfficiencyDistance;
    double SmartMoveProb;
    int UncertaintyCount;



private:

    mutable MEMORY_POOL<ROCKSAMPLE_STATE> MemoryPool;
	double QValues[20000];
	ROCKSAMPLE_STATE qstate1;
	ROCKSAMPLE_STATE qstate2;
	std::map<ROCKSAMPLE_BELIEF,VNODE*> belief_map;
};

#endif
