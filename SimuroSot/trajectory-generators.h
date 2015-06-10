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
inline Trajectory* circleGenerator(double x, double y, double r, double startTheta, double f) {
    function<double(double)> xfunc = [=](double t)->double {
        return (r*sin(2*PI*f*t + startTheta)+x)/fieldXConvert;
    };
    function<double(double)> yfunc = [=](double t)->double {
        return (r*cos(2*PI*f*t + startTheta)+y)/fieldXConvert;
    };
    return new Trajectory(xfunc, yfunc);
}

inline SplineTrajectory *cubic(Pose start, Pose end, double vls, double vrs, double vle, double vre, vector<Pose> midPoints = vector<Pose>()) {
    CubicSpline *p = new CubicSpline(start, end, midPoints);
//    p->maxk();
    SplineTrajectory *st = new SplineTrajectory(p, vls, vrs, vle, vre);
    return st;
}

}
