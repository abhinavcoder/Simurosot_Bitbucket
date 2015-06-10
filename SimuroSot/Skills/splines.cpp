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
//    vector<double> a(4,0), b(4,0);
//    a[3] = d * cos(th2) + d * cos(th1) - 2 * (x2 - x1);
//    a[2] = 3 * (x2 - x1) - d * cos(th2) - 2 * d * cos(th1);
//    a[1] = d * cos(th1);
//    a[0] = x1;
//    b[3] = d * sin(th2) + d * sin(th1) - 2 * (y2 - y1);
//    b[2] = 3 * (y2 - y1) - d * sin(th2) - 2 * d * sin(th1);
//    b[1] = d * sin(th1);
//    b[0] = y1;
//    // make them cm from strategy coordinates
//    for (int i = 0; i < 4; i++) {
//        a[i] = a[i]/Constants::fieldXConvert;
//        b[i] = b[i]/Constants::fieldXConvert;
//    }
//    p = ParamPoly(a, b);

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

double fn1 (double u, void * params)
{
  CubicSpline *s = static_cast<CubicSpline*>(params);
  return -1.0*(s->k(u))*(s->k(u));
}

double fn1 (double u, const CubicSpline *s)
{
  return -1.0*(s->k(u))*(s->k(u));
}

double kd(double u, void *params){

    CubicSpline *s = static_cast<CubicSpline*>(params);
    double yddd = s->yddd(u);
    double xddd = s->xddd(u);
    double xd = s->xd(u);
    double xdd = s->xdd(u);
    double yd = s->yd(u);
    double ydd = s->ydd(u);
    return ((yddd * xd - xddd * yd) / pow((xd * xd + yd * yd), 1.5)) - ((3 * (xd * ydd - xdd * yd) * (xd * xdd + yd * ydd)) / pow((xd * xd + yd * yd), 2.5));
}

double kd_neg(double u, void *params) {
    CubicSpline *s = static_cast<CubicSpline*>(params);
    double yddd = s->yddd(u);
    double xddd = s->xddd(u);
    double xd = s->xd(u);
    double xdd = s->xdd(u);
    double yd = s->yd(u);
    double ydd = s->ydd(u);
    return -((yddd * xd - xddd * yd) / pow((xd * xd + yd * yd), 1.5)) + ((3 * (xd * ydd - xdd * yd) * (xd * xdd + yd * ydd)) / pow((xd * xd + yd * yd), 2.5));
}

double kd_df(double u, void *params) {

    CubicSpline *s = static_cast<CubicSpline*>(params);
    double h = 0.0001;
    double xdddd = 0.;
    double ydddd = 0.;
    double yddd = s->yddd(u);
    double xddd = s->xddd(u);
    double xd = s->xd(u);
    double xdd = s->xdd(u);
    double yd = s->yd(u);
    double ydd = s->ydd(u);
    double p1 = - (6 * (yddd * xd - xddd * yd) * (xd * xdd + yd * ydd)) / (pow((xd * xd + yd * yd), 2.5));
    double p2 = (xd * ydd - yd * xdd) * ((15 * pow((xd * xdd + yd * ydd), 2) / pow((xd * xd + yd * yd), 3.5)) - 3 * (xdd * xdd + xddd * xd + ydd * ydd + yddd * yd) / pow((xd * xd + yd * yd), 2.5));
    double p3 = (-0*xdddd * yd - xddd * ydd + yddd * xdd + 0*ydddd * xd) / pow((xd * xd + yd * yd), 1.5);
    return (p1 + p2 + p3);
}

double kd_neg_df(double u, void *params) {
    CubicSpline *s = static_cast<CubicSpline*>(params);
    double xdddd = 0.;
    double ydddd = 0.;
    double yddd = s->yddd(u);
    double xddd = s->xddd(u);
    double xd = s->xd(u);
    double xdd = s->xdd(u);
    double yd = s->yd(u);
    double ydd = s->ydd(u);
    double p1 = - (6 * (yddd * xd - xddd * yd) * (xd * xdd + yd * ydd)) / (pow((xd * xd + yd * yd), 2.5));
    double p2 = (xd * ydd - yd * xdd) * ((15 * pow((xd * xdd + yd * ydd), 2) / pow((xd * xd + yd * yd), 3.5)) - 3 * (xdd * xdd + xddd * yd + ydd * ydd + yddd * yd) / pow((xd * xd + yd * yd), 2.5));
    double p3 = (-xdddd * yd - xddd * ydd + yddd * xdd + ydddd * xd) / pow((xd * xd + yd * yd), 1.5);
    return -(p1 + p2 + p3);
}
void kd_fdf(double u, void *params, double *y, double *dy) {

    CubicSpline *s = static_cast<CubicSpline*>(params);
    double xdddd = 0.;
    double ydddd = 0.;
    double yddd = s->yddd(u);
    double xddd = s->xddd(u);
    double xd = s->xd(u);
    double xdd = s->xdd(u);
    double yd = s->yd(u);
    double ydd = s->ydd(u);
    *y = ((yddd * xd - xddd * yd) / pow((xd * xd + yd * yd), 1.5)) - ((3 * (xd * ydd - xdd * yd) * (xd * xdd + yd * ydd)) / pow((xd * xd + yd * yd), 2.5));

    double p1 = - (6 * (yddd * xd - xddd * yd) * (xd * xdd + yd * ydd)) / (pow((xd * xd + yd * yd), 2.5));
    double p2 = (xd * ydd - yd * xdd) * ((15 * pow((xd * xdd + yd * ydd), 2) / pow((xd * xd + yd * yd), 3.5)) - 3 * (xdd * xdd + xddd * yd + ydd * ydd + yddd * yd) / pow((xd * xd + yd * yd), 2.5));
    double p3 = (-xdddd * yd - xddd * ydd + yddd * xdd + ydddd * xd) / pow((xd * xd + yd * yd), 1.5);
    *dy = (p1 + p2 + p3);
}

void kd_neg_fdf(double u, void *params, double *y, double *dy) {

    CubicSpline *s = static_cast<CubicSpline*>(params);
    double xdddd = 0.;
    double ydddd = 0.;
    double yddd = s->yddd(u);
    double xddd = s->xddd(u);
    double xd = s->xd(u);
    double xdd = s->xdd(u);
    double yd = s->yd(u);
    double ydd = s->ydd(u);
    *y = -((yddd * xd - xddd * yd) / pow((xd * xd + yd * yd), 1.5)) + ((3 * (xd * ydd - xdd * yd) * (xd * xdd + yd * ydd)) / pow((xd * xd + yd * yd), 2.5));

    double p1 = - (6 * (yddd * xd - xddd * yd) * (xd * xdd + yd * ydd)) / (pow((xd * xd + yd * yd), 2.5));
    double p2 = (xd * ydd - yd * xdd) * ((15 * pow((xd * xdd + yd * ydd), 2) / pow((xd * xd + yd * yd), 3.5)) - 3 * (xdd * xdd + xddd * yd + ydd * ydd + yddd * yd) / pow((xd * xd + yd * yd), 2.5));
    double p3 = (-xdddd * yd - xddd * ydd + yddd * xdd + ydddd * xd) / pow((xd * xd + yd * yd), 1.5);
    *dy = -(p1 + p2 + p3);
}

double CubicSpline::maxk(double *u_low) const
{
    using namespace alglib;
    real_2d_array tblx, tbly;
    ae_int_t nx, ny;
    alglib::spline1dunpack(splineX, nx, tblx);
    alglib::spline1dunpack(splineY, ny, tbly);
    assert (nx == ny);
    double maxk = 0;
    double maxk_u = 0;
    // iterate through each segment, find the maxk
    std::cout << "new call:";
    for (int i = 0; i < nx-1; i++) {
        double u_low = tblx[i][0], u_high = tblx[i][1];
        assert(tbly[i][0] == u_low && tbly[i][1] == u_high);
        // get coefficients
        double a[4], b[4]; // a = x coeff, b = y coeff
        for (int j = 0; j < 4; j++) {
            a[j] = tblx[i][j+2];
            b[j] = tbly[i][j+2];
        }
        std::cout << "coeff (x), (y) =  x(t)=" << a[3] <<"t^3 + "<< a[2] <<"t^2 + "<< a[1] <<"t + "<< a[0] <<", y(t)=" <<
                    b[3] <<"t^3 + "<< b[2] <<"t^2 + "<< b[1] <<"t + "<< b[0] << "ulow, uhigh=" << u_low << u_high;
        // get k value at beginning
        if (fabs(this->k(u_low)) > maxk) {
            maxk = fabs(this->k(u_low));
            maxk_u = u_low;
        }
        // get k value at end of this spline component
        if (fabs(this->k(u_high)) > maxk) {
            maxk = fabs(this->k(u_high));
            maxk_u = u_high;
        }
        // get u value for which xd*ydd-yd*xdd is extremum
        // this doesn't actually find extrema of k, but i think it should be good enough
        // solution is:
        // u = (b1*a3-a1*b3)/(2*(a2*b3-b2*a3))
        if (a[2]*b[3]-b[2]*a[3] != 0) {
            // the spline in alglib takes input t = u-u_low
            double t_ext = (b[1]*a[3]-a[1]*b[3])/2./(a[2]*b[3]-b[2]*a[3]);
            if (t_ext >= 0 && t_ext <= u_high-u_low) {
                double u_ext = t_ext + u_low;
                if (fabs(this->k(u_ext)) > maxk) {
                    maxk = fabs(this->k(u_ext));
                    maxk_u = u_ext;
                }
            }
        }
        // above code doesn't seem to work, trying minimization.
//        {
//              int status;
//              int iter = 0, max_iter = 100;
//              const gsl_min_fminimizer_type *T;
//              gsl_min_fminimizer *s;
//              gsl_function F;

//              F.function = &fn1;
//              F.params = const_cast<CubicSpline*>(this);

//              T = gsl_min_fminimizer_brent;
//              s = gsl_min_fminimizer_alloc (T);
//              double a = u_low, b = u_high;
//              double at=a, bt=b;
//              //if(this->k(b)*this->k(b) < this->k(a)*this->k(a)){/*exit(9);*/a=u_high;b=u_low;}
//              double m,m_prev;
////              do{
////                    if((fn1(m,this) > this->k(at)*this->k(at)))at=(bt+at)/2;
////                    if((this->k(m)*this->k(m) < this->k(bt)*this->k(bt)))bt=(at+bt)/2;
////                    m = (at+bt)/2;
////                    std::cout << "in the loop " << m << " " << at << " " << bt << " " << a << " " << b;
////                    if(m==at || m==bt)return this->k(m)*this->k(m);
////                    if(abs(m_prev-m)<1e-3)return this->k(m)*this->k(m);
////                    m_prev = m;

////              }while((this->k(m)*this->k(m) > this->k(a)*this->k(a)) && (this->k(m)*this->k(m) < this->k(b)*this->k(b)));
////                a=at;b=bt;
//              //if((-fabs(this->k(m)) > -fabs(this->k(b))) || (-fabs(this->k(m)) < -fabs(this->k(a)))){exit(91);}

////              MNBRAK(a,b,&x3,&f1,&f2,&f3,this);
////              m=x3;

#define SCALE 1.618
//        double fa = f1(a, s);
//        double fb = f1(b, s);
//        double c = b + SCALE * (b-a);
//        double fc = f1(c,s);
//        while (fb > fc)
//        {
//            a = b; fa = fb;
//            b = c; fb = fc;
//            c = b + SCALE * (b - a);
//            fc = f1(c,s);
//        }
//        m=b;b=c;
//              if(m<a || m>b)exit(9);
//              if (gsl_min_fminimizer_set (s, &F, m, a, b) != GSL_FAILURE) {
//                  //printf ("using %s method\n",
//                  //        gsl_min_fminimizer_name (s));



//                  do
//                    {
//                      iter++;
//                      status = gsl_min_fminimizer_iterate (s);

//                      m = gsl_min_fminimizer_x_minimum (s);
//                      a = gsl_min_fminimizer_x_lower (s);
//                      b = gsl_min_fminimizer_x_upper (s);

//                      status
//                        = gsl_min_test_interval (a, b, 0.01, 0.0);

////                      if (status == GSL_SUCCESS)
////                        printf ("Converged:\n");

////                      printf ("%5d [%.7f, %.7f] "
////                              "%.7f %+.7f %.7f\n",
////                              iter, a, b,
////                              m, m, b - a);
//                    }
//                  while (status == GSL_CONTINUE && iter < max_iter);

//                  gsl_min_fminimizer_free (s);
//                  if (status == GSL_SUCCESS) {
//                      if (fabs(this->k(m)) > maxk) {
//                          maxk = fabs(this->k(m));
//                          maxk_u = m;
//                      }
//                  }
//              }
//        }


    }

    double tempmaxk=0;
    float umax;

    for(float u=0;u<1;u+=0.001){
        //std::cout << "Curvature at u as " << u << " = " << std::abs(this->k(u));
        if(std::abs(this->k(u)) > tempmaxk){
            tempmaxk = std::abs(this->k(u));
            umax = u;
        }
    }


    //Newton-Rhapson Approach for finding out max curvature
            const gsl_root_fdfsolver_type *T;
            gsl_root_fdfsolver *sf;
            int status;
            int iter = 0, max_iter = 100;

            double x0, x = umax;


            gsl_function_fdf F;
            F.f = &kd;
            F.df = &kd_df;
            F.fdf = &kd_fdf;
            F.params = const_cast<CubicSpline*>(this);

            T = gsl_root_fdfsolver_newton;
              sf = gsl_root_fdfsolver_alloc (T);
              gsl_root_fdfsolver_set (sf, &F, x);

              //printf("Using %s method\n", gsl_root_fdfsolver_name(sf));
              //printf("%-5s %10s %10s %10s\n", "iter", "root", "error", "err(est)");

              do
                 {
                   iter++;
                   status = gsl_root_fdfsolver_iterate (sf);
                   x0 = x;
                   x = gsl_root_fdfsolver_root (sf);
                   status = gsl_root_test_delta (x, x0, 0, 1e-2);

                   //if(status== GSL_SUCCESS)printf("Converged:\n");
                   //printf("%5d %10.7f %+10,7f %10.7f\n", iter, x, x-r_expected, x- x0);
                 }
                while (status == GSL_CONTINUE && iter < max_iter);

                if(std::abs(this->k(x)) > maxk)maxk = std::abs(this->k(x));

    //            gsl_function_fdf Fneg;
    //            Fneg.f = &kd_neg;
    //            Fneg.df = &kd_neg_df;
    //            Fneg.fdf = &kd_neg_fdf;
    //            Fneg.params = const_cast<CubicSpline*>(this);
    //            iter =0; x = this->k(0.5);

    //            do
    //               {
    //                 iter++;
    //                 status = gsl_root_fdfsolver_iterate (sf);
    //                 x0 = x;
    //                 x = gsl_root_fdfsolver_root (sf);
    //                 status = gsl_root_test_delta (x, x0, 0, 1e-2);

    //                 //if(status== GSL_SUCCESS)printf("Converged:\n");
    //                 //printf("%5d %10.7f %+10,7f %10.7f\n", iter, x, x-r_expected, x- x0);
    //               }
    //              while (status == GSL_CONTINUE && iter < max_iter);

                gsl_root_fdfsolver_free (sf);
    //            if(abs(x) > maxk)maxk = abs(x);

    std::cout << "maxk = " << maxk << ", tempmaxk = " << tempmaxk << "umax = " << umax;

    if (u_low)
        *u_low = maxk_u;
    return maxk;
}
