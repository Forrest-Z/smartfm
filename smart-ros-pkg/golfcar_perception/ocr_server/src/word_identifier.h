#include "ros/ros.h"
#include <cstdlib>
#include <vector>
#include <stdlib.h>
#include <float.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>


class word_identifier {

public:
  word_identifier()
  {
  }
  ~word_identifier()
  {
  }

public:
  bool identify (int* feature_vector, int vector_length, std::string &response);
  bool test();
};
