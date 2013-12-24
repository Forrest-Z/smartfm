#ifndef MCTS_H
#define MCTS_H

#include "simulator.h"
#include "node.h"
#include "statistic.h"
#include "battleship.h"
#include "rocksample.h"
#include "pedestrian.h"
#include "pedestrian_dynamic.h"
#include "pedestrian_changelane.h"
#include "pocman.h"
#include <fstream>

class MCTS
{
public:

    struct PARAMS
    {
		PARAMS();
        int Verbose;
        int MaxDepth;
        int NumSimulations;
        int NumStartStates;
        bool UseTransforms;
        int NumTransforms;
        int MaxAttempts;
        int ExpandCount;
        double ExplorationConstant;
        bool UseRave;
        double RaveDiscount;
        double RaveConstant;
        bool DisableTree;
		bool UseQmdp;
		bool UseTime;
    };

    MCTS(const SIMULATOR& simulator, const PARAMS& params);
    ~MCTS();

    int SelectAction();
    bool Update(int action, int observation, double reward);

    void UCTSearch();
    void RolloutSearch();

    double Rollout(STATE& state);
	double User_Rollout(STATE& state);

    const BELIEF_STATE& BeliefState() const { return Root->Beliefs(); }
    const HISTORY& GetHistory() const { return History; }
    const SIMULATOR::STATUS& GetStatus() const { return Status; }
    void ClearStatistics();
    void DisplayStatistics(std::ostream& ostr) const;
    void DisplayValue(int depth, std::ostream& ostr) const;
    void DisplayPolicy(int depth, std::ostream& ostr) const;

	VNODE* VMerge(VNODE *n1, VNODE*n2);
	QNODE * QMerge(QNODE *n1, QNODE*n2);
	void Merge(VNODE *n1, VNODE*n2);

    static void UnitTest();
    static void InitFastUCB(double exploration);
    TESTPROBLEM* test;
	ROCKSAMPLE* rockproblem;
	TIGER* tigerproblem;
	TESTPROBLEM* testproblem;
	PEDESTRIAN* pedproblem;
	PEDESTRIAN_DYNAMIC*pedproblem_d;
	PEDESTRIAN_CHANGELANE*pedproblem_c;
	int MaxTreeDepth;
	std::ofstream depth_out;
	std::ofstream listen_out;
	std::ofstream openl_out;
	std::ofstream openr_out;
	std::ofstream count_out;
	std::ofstream ucbvalues_out;
	std::ofstream merge_out;
	int o,l,r,sum;
	std::vector<VNODE*> root_list;
    VNODE* Root;
private:

    const SIMULATOR& Simulator;
    int TreeDepth, PeakTreeDepth;
    PARAMS Params;

	VNODE* Rollout_Root;
    HISTORY History;
    SIMULATOR::STATUS Status;

    STATISTIC StatTreeDepth;
    STATISTIC StatRolloutDepth;
    STATISTIC StatTotalReward;

    int GreedyUCB(VNODE* vnode, bool ucb) ;//originally declared with const at the end
    int SelectRandom() const;
    double SimulateV(STATE& state, VNODE* vnode);
    double SimulateQ(STATE& state, QNODE& qnode, int action);
    void AddRave(VNODE* vnode, double totalReward);
    VNODE* ExpandNode(const STATE* state);
    void AddSample(VNODE* node, const STATE& state);
    void AddTransforms(VNODE* root, BELIEF_STATE& beliefs);
    STATE* CreateTransform() const;
    void Resample(BELIEF_STATE& beliefs);

    // Fast lookup table for UCB
    static const int UCB_N = 10000, UCB_n = 100;
    static double UCB[UCB_N][UCB_n];
    static bool InitialisedFastUCB;

    double FastUCB(int N, int n, double logN) const;

    static void UnitTestGreedy();
    static void UnitTestUCB();
    static void UnitTestRollout();
    static void UnitTestSearch(int depth);
};

#endif // MCTS_H
