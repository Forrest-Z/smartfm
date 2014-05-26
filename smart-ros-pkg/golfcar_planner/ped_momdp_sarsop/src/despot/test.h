#ifndef CODE_TEST_H
#define CODE_TEST_H

#include <iostream>
#include <vector>

#include "pomdp.h"
#include "policy.h"
#include "node.h"
#include "random_streams.h"
#include "lower_bound.h"

#include "util/random.h"
#include "util/seeds.h"
#include "util/logging.h"

#include "despot.h"
#include "despotstar.h"

#include "problems/bridge.h"
#include "problems/tiger.h"
#include "problems/tag.h"
#include "problems/lasertag.h"

using namespace std;

void TestRandom();

void TestTree();

void SimulateModel(DSPOMDP * model, ostream& os);

void TestModel();
void TestDespot();

void CompareTagImplementations(int argc, char* argv[]);
void GenerateRandomInitialConfigs(int argc, char* argv[]);

#endif
