#include "stdafx.h"
#include "splines.hpp"
#include "alglib/interpolation.h"
#include "gsl/gsl_min.h"
#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_roots.h>
#include <cmath>
#include <vector>
#include "../common/include/geometry.hpp"

#define Point2D Vector2D
#include <iostream>
#include <stdio.h>
using namespace std;

CubicSpline::CubicSpline(Pose start, Pose end, std::vector<Pose> midPoints)
{
    double d = sqrt((start.x() - end.x())*(start.x() - end.x()) + (start.y() - end.y())*(start.y() - end.y()));
    d = d/fieldXConvert;
    double x1 = start.x()/fieldXConvert;
    double x2 = end.x()/fieldXConvert;
    double y1 = start.y()/fieldXConvert;
    double y2 = end.y()/fieldXConvert;
    double th1 = start.theta();
    double th2 = end.theta();
    {
        using namespace alglib;
        double n = midPoints.size()+2; // number of points to interpolate on
        vector<double> x(n,0), y(n,0), u(n,0);
        x[0] = x1;
        x[n-1] = x2;
        y[0] = y1;
        y[n-1] = y2;
        for (int i = 0; i < n; i++) {
            u[i] = i/(double)(n-1);
        }
        for (int i = 1; i < n-1; i++ ) {
            x[i] = midPoints[i-1].x()/fieldXConvert;
            y[i] = midPoints[i-1].y()/fieldXConvert;
        }
        alglib::real_1d_array AU, AY, AX;
        AU.setcontent(u.size(), &(u[0]));
        AY.setcontent(y.size(), &(y[0]));
        AX.setcontent(x.size(), &(x[0]));
        spline1dbuildcubic(AU, AX, u.size(), 1, d*cos(th1), 1, d*cos(th2), splineX);
        spline1dbuildcubic(AU, AY, u.size(), 1, d*sin(th1), 1, d*sin(th2), splineY);
    }
}

double CubicSpline::x(double u) const
{
    double s, ds, d2s;
    alglib::spline1ddiff(splineX, u, s, ds, d2s);
    return s;
}

double CubicSpline::y(double u) const
{
    double s, ds, d2s;
    alglib::spline1ddiff(splineY, u, s, ds, d2s);
    return s;
}

double CubicSpline::xd(double u) const
{
    double s, ds, d2s;
    alglib::spline1ddiff(splineX, u, s, ds, d2s);
    return ds;
}

double CubicSpline::yd(double u) const
{
    double s, ds, d2s;
    alglib::spline1ddiff(splineY, u, s, ds, d2s);
    return ds;
}

double CubicSpline::xdd(double u) const
{
    double s, ds, d2s;
    alglib::spline1ddiff(splineX, u, s, ds, d2s);
    return d2s;
}

double CubicSpline::ydd(double u) const
{
    double s, ds, d2s;
    alglib::spline1ddiff(splineY, u, s, ds, d2s);
    return d2s;
}

double CubicSpline::xddd(double u) const {
    using namespace alglib;
    real_2d_array tblx;
    ae_int_t nx;
    double ret = 0.;
    alglib::spline1dunpack(splineX, nx, tblx);
    for (int i = 0; i < nx - 1; i++) {
        double u_low = tblx[i][0], u_high = tblx[i][1];
        if (u >= u_low && u <= u_high) {
             ret = 6 * tblx[i][5];
        }
    }
    return ret;
}

double CubicSpline::yddd(double u) const {
    using namespace alglib;
    real_2d_array tbly;
    int ny;
    double ret = 0.;
    alglib::spline1dunpack(splineY, ny, tbly);
    for (int i = 0; i < ny - 1; i++) {
        double u_low = tbly[i][0], u_high = tbly[i][1];
        if (u >= u_low && u <= u_high) {
             ret = 6 * tbly[i][5];
        }
    }
    return ret;
}