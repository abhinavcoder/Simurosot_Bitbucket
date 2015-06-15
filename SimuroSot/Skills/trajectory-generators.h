#include <tracking.hpp>
#include <functional>
#include <math.h>
#include "pose.h"
#include "velocity-profile.hpp"
#include "trajectory.hpp"
#include "splines.hpp"
//#include "controlpoint-optimization.hpp"
using namespace std;
namespace TrajectoryGenerators {
inline SplineTrajectory *cubic(Pose start, Pose end, double vls, double vrs, double vle, double vre, vector<Pose> midPoints = vector<Pose>()) {
    CubicSpline *p = new CubicSpline(start, end, midPoints);
    SplineTrajectory *st = new SplineTrajectory(p, vls, vrs, vle, vre);
    return st;
}

}
