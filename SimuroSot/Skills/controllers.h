// definitions of some point to point controllers. trajectory generators are in trajectory-generators.hpp
#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include "pose.h"
#include <deque>
#include "tracking.hpp"
#include <assert.h>
#include "../common/include/geometry.hpp"
using namespace std;

const int timeLCMs = 16;
const double timeLC = timeLCMs*0.001;

#endif // CONTROLLERS_H
