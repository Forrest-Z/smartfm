#include <pedestrian_dynamic.h>
#include "mcts.h"
#include "pedestrian_dynamic.h"


class Executer
{
public:
	Executer();
        void Step(int action,int observation);
	int  GetAction();
	void Init();
	
	PEDESTRIAN_DYNAMIC*sim;
	MCTS*mcts;
};
