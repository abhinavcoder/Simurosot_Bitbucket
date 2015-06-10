#include "stdafx.h"
#include "arclength-param.hpp"
#include "arclength-param.hpp"
#include <assert.h>
#include <time.h>
#include "alglib/interpolation.h"
#include "gsl/gsl_roots.h"
#include "gsl/gsl_errno.h"
#include "gsl/gsl_math.h"
#include "gsl/gsl_blas.h"
#include <algorithm>
#include <iostream>

namespace Integration {
gsl_integration_workspace * w = NULL;
double f(double u, void *integrand) {
  Integrand *in = (Integrand*) integrand;
  return (*in)(u);
}

struct func_params
{
    double s;
    Spline& p;
};

double func(double u, void *params) {

    struct func_params *ps = (func_params *) params;
    return integrate(ps->p, 0, u)-ps->s;
}

double func_df(double u, void *params) {

    struct func_params *ps = (func_params *) params;

  Spline &pin = (Spline&) ps->p;
  return pin(u);
}

void func_fdf(double u, void *params, double *y, double *dy) {

    struct func_params *ps = (func_params *) params;
  Spline &pin = (Spline&) ps->p;
  *y = integrate(ps->p, 0, u)-ps->s;

  *dy = pin(u);
}

double integrate(Integrand& i, double s, double e) {
  double result, error;
  if (w == NULL) {
      w = gsl_integration_workspace_alloc (1000);
  }
  gsl_function F;
  F.function = f;
  F.params = &i;
  gsl_integration_qags (&F, s, e, 0, 1e-7, 1000,
                        w, &result, &error);

  return result;
}

const gsl_root_fdfsolver_type *T;
gsl_root_fdfsolver *sf;

double optimise_u(Spline& p, double s, double e, bool is_newton=true){

    int status;
      int iter = 0, max_iter = 100;

        double x0, x = s/e, r_expected=s;


    gsl_function_fdf F;
    F.f = &func;
    F.df = &func_df;
    F.fdf = &func_fdf;



    func_params fp = {s,p};
    F.params = &fp;

    if(is_newton)T = gsl_root_fdfsolver_newton;
    else T = gsl_root_fdfsolver_steffenson;
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
           status = gsl_root_test_delta (x, x0, 0, 1e-3);

           //if(status== GSL_SUCCESS)printf("Converged:\n");
           //printf("%5d %10.7f %+10,7f %10.7f\n", iter, x, x-r_expected, x- x0);
         }
        while (status == GSL_CONTINUE && iter < max_iter);

        gsl_root_fdfsolver_free (sf);

       return x;
}

double arclenddu(double u, void *params) {

    Spline *ps = (Spline*) params;
    double xd = ps->xd(u);
    double xdd = ps->xdd(u);
    double yd = ps->yd(u);
    double ydd = ps->yd(u);

    double f = (xd*xdd + yd*ydd);
    return f;
}

//approximating using C.D. methods for x'''(u)
//also try using spline1dunpack
double arclenddu_df(double u, void *params) {

    Spline *ps = (Spline*) params;      //possible improvement- return all xd, xdd in one call
    double xd = ps->xd(u);
    double xdd = ps->xdd(u);
    double ydd = ps->ydd(u);
    double yd = ps->yd(u);
    double xddd, yddd;
    double h = 0.01;

    if(u==1){
        xddd = (xdd-ps->xdd(u-h))/h;
        yddd = (ydd-ps->ydd(u-h))/h;
    }
    else if(u==0){
        xddd = (ps->xdd(u+h)-xdd)/h;
        yddd = (ps->ydd(u+h)-ydd)/h;
    }
    else{
        xddd = (ps->xdd(u+h)-ps->xdd(u-h))/(2*h);
        yddd = (ps->ydd(u+h)-ps->ydd(u-h))/(2*h);
    }
//    double f = ((xdd*xdd + xddd*xd + ydd*ydd + yddd*yd)/(sqrt(xd*xd + yd*yd)))
//            - (pow((xd*xdd + yd*ydd),2)/(pow(xd*xd + yd*yd,3/2)));

    double f = xdd*xdd + xddd*xd + ydd*ydd + yddd*yd;
    return f;
}

void arclenddu_fdf(double u, void *params, double *y, double *dy) {

   Spline *ps = (Spline*) params;
   *y = ps->xd(u)*ps->xdd(u) + ps->yd(u)*ps->ydd(u);

   double xd = ps->xd(u);
   double xdd = ps->xdd(u);
   double ydd = ps->ydd(u);
   double yd = ps->yd(u);
   double xddd, yddd;
   double h = 0.01;

   if(u==1){
       xddd = (xdd-ps->xdd(u-h))/h;
       yddd = (ydd-ps->ydd(u-h))/h;
   }
   else if(u==0){
       xddd = (ps->xdd(u+h)-xdd)/h;
       yddd = (ps->ydd(u+h)-ydd)/h;
   }
   else{
       xddd = (ps->xdd(u+h)-ps->xdd(u-h))/(2*h);
       yddd = (ps->ydd(u+h)-ps->ydd(u-h))/(2*h);
   }
//   *dy = ((xdd*xdd + xddd*xd + ydd*ydd + yddd*yd)/(sqrt(xd*xd + yd*yd)))
//           - (pow((xd*xdd + yd*ydd),2)/(pow(xd*xd + yd*yd,3/2)));

   *dy = xdd*xdd + xddd*xd + ydd*ydd + yddd*yd;
}

double find_u(Spline &p, float i){

    int status;
    int iter = 0, max_iter = 100;

    double x0, x = i;
    gsl_function_fdf F;

    F.f = &arclenddu;
    F.df = &arclenddu_df;
    F.fdf = &arclenddu_fdf;
    F.params = &p;

      T = gsl_root_fdfsolver_newton;
      sf = gsl_root_fdfsolver_alloc (T);
       gsl_root_fdfsolver_set (sf, &F, x);

      do{
           iter++;
           status = gsl_root_fdfsolver_iterate (sf);
           x0 = x;
           x = gsl_root_fdfsolver_root (sf);
           status = gsl_root_test_delta (x, x0, 0, 1e-2);

           //if(status== GSL_SUCCESS)printf("Converged:\n");
        }
      while (status == GSL_CONTINUE && iter < max_iter);

      gsl_root_fdfsolver_free (sf);

      return x;
}

vector<double> getInflectionPoints(Spline &p, double start_u, double end_u){

    vector<double> arr;
    for(float i=0;i<=1;i+=0.1){
        arr.push_back(find_u(p,i)); //results are not as good as brute force,but ok

       //std::cout << "  " << arr[10*(int)i];
    }
    sort(arr.begin(), arr.end());

//    printf("||||   number of points = %d", arr.size());
//    for(vector<double>::iterator it = arr.begin(); it!=arr.end(); ++it){
//        printf("\t %f ",*it);
//    }
    vector<double>::iterator it = arr.begin();
    while(it!=arr.end()-1){
        if(*it > 1 || *it < 0){
            it = arr.erase(it);
        }
        else if(abs(*it - *(it+1)) <= 1e-2){
            *(it+1) = (*(it+1)+*it)/2;
            it = arr.erase(it);
            it--;
        }
        else{
            it++;
        }
    }
    if(*it >1 || *it <0)arr.erase(it);

    printf("\n||||   number of points = %d", arr.size());
    for(vector<double>::iterator it = arr.begin(); it!=arr.end(); ++it){
        printf("\t %f ",*it);
    }

    printf("\n");
    //Original s''(t) values
//    for(float u=0;u<=1;u+=0.0001){
//        double f = p.xd(u)*p.xdd(u) + p.yd(u)*p.ydd(u);
//        if(abs(f)<100)
//            printf("\ns''(t) values and pos - %f %f ", f,u);
//    }

    return arr;
}

double s_formula(Spline &p, double b){
    if(b==0)return 0.0;
    double f = (b/2)*(0.55555*p(0.8872925*b) + 0.88888*p(b/2) + 0.55555*p(0.1127015*b));
    return f;
}

void get_bezier(Spline &p, gsl_matrix_view *BVx, gsl_matrix_view *BVy, double start_u, double end_u){

    //double s_1by3 = Integration::s_formula(p,start_u + (1/3)*(end_u - start_u)) - Integration::s_formula(p,start_u);
    //double s_2by3 = Integration::s_formula(p,start_u + (2/3)*(end_u - start_u)) - Integration::s_formula(p,start_u);
    //double s_1 = Integration::s_formula(p,end_u) - Integration::s_formula(p,start_u);


    double s_1by3 = integrate(p, 0,start_u + (1.0/3)*(end_u - start_u)) - integrate(p, 0,start_u);
    double s_2by3 = integrate(p, 0,start_u + (2.0/3)*(end_u - start_u)) - integrate(p, 0,start_u);
    double s_1 = integrate(p, 0,end_u) - integrate(p, 0,start_u);
    double sm_1by3 = s_1by3/s_1;
    double sm_2by3 = s_2by3/s_1;

    std::cout << "Value of s and s1 " << s_1by3 << " " << s_2by3 << "" << s_1;
    double v1y = (18*sm_1by3 - 9*sm_2by3 + 2)/6;
    double v2y = (-9*sm_1by3 + 18*sm_2by3 -5)/6;

    double b[] = {-1,3,-3,1,3,-6,3,0,-3,3,0,0,1,0,0,0};
    double vx[] = {0,start_u + (1/3)*(end_u - start_u), start_u + (2/3)*(end_u - start_u), end_u};
    double vy[] = {0, v1y, v2y, 1};

    /*
    double b2[] = {1, -10, 45, -120, 210, -252, 210, -120, 45, -10, 1,
                   -10, 90, -360, 840, -1260, 1260, -840, 360, -90, 10, 0,
                   45, -360, 1260, -2520, 3150, -2520, 1260, -360, 45, 0, 0,
                   -120, 840, -2520, 4200, -4200, 2520, -840, 120, 0, 0, 0,
                   210, -1260, 3150, -4200, 3150, -1260, 210, 0, 0, 0, 0,
                   -252, 1260, -2520, 2520, -1260, 252, 0, 0, 0, 0, 0,
                   210, -840, 1260, -840, 210, 0, 0, 0, 0, 0, 0,
                   -120, 360, -360, 120, 0, 0, 0, 0, 0, 0, 0,
                   45, -90, 45, 0, 0, 0, 0, 0, 0, 0, 0,
                   -10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                  };
    */

    gsl_matrix_view B = gsl_matrix_view_array(b, 4,4);
    gsl_matrix_view Vx = gsl_matrix_view_array(vx, 4,1);
    gsl_matrix_view Vy = gsl_matrix_view_array(vy, 4,1);


    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &B.matrix, &Vx.matrix, 0,&BVx->matrix);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &B.matrix, &Vy.matrix, 0,&BVy->matrix);

}

int span;
vector<double> infpts;

int invspan;
double invbvx[] = {0.0, 0.0, 0.0, 0.0};
double invbvy[] = {0.0, 0.0, 0.0, 0.0};
gsl_matrix_view IBVx = gsl_matrix_view_array(invbvx, 4,1);
gsl_matrix_view IBVy = gsl_matrix_view_array(invbvy, 4,1);
vector<double> invinfpts;

void refreshMatrix(){
    for(int i=0;i<4;i++){
        invbvx[i]=invbvy[i]=0;
    }
    infpts.clear();
}

alglib::spline1dinterpolant splineSU;
void computeSplineApprox(Spline &p){

    using namespace alglib;
    double n = 10; // number of points to interpolate on
    vector<double> s(n,0), u(n,0);

    for(int i=0;i<n;i++){
        u[i] = i*1.0/n;
        s[i] = integrate(p,0,u[i]);
        //std::cout << "U and S are " << u[i] << s[i] << "\n\n";
    }

    //if(ut2 < ut1){double temp=ut2;ut2=ut1;ut1=temp;}
    alglib::real_1d_array AU, AS;
    AS.setcontent(s.size(), &(s[0]));
    AU.setcontent(u.size(), &(u[0]));
    //spline1dbuildcubic(AS, AU, u.size(), 1, ut2, 1, ut1, splineSU);
    spline1dbuildcubic(AS, AU, splineSU);

//    double ut,dut,d2ut;
//    for(int st=0;st<n;st++){
//        spline1ddiff(splineSU, 0.9*s[st], ut, dut, d2ut);
//        std::cout << "UT and ST are " << ut << 0.9*s[st] << "\n";
//    }
}

/* function to compute u(s)
    v1y = (ce+2b-e-bf)/(bd-ae);
    v2y = (d-2a-cd+fa)/(bd-ae);
    for equations as -
    a*v1y + b*v2y + c = 1
    d*v1y + e*v2y + f = 2
    where,
    a = 9*(s(1/3) + s(1/3)^3 - 2*s(1/3)^2)
    b = 9*(s(1/3)^2 - s(1/3)^3)
    c = 3*(s(1/3)^3)
    d = 9*(s(2/3) + s(2/3)^3 - 2*s(2/3)^2)
    e = 9*(s(2/3)^2 - s(2/3)^3)
    f = 3*(s(2/3)^3)

    Simplified:
    v1y:
        Nr - 3*s*s*s1*s1*(s-s1) + 2*s*s(1-s) - s1*s1*(1-s1)
        Dr - 9*s*s1*(1-s1)(1-s)(s-s1)

    v2y:
        Nr - s1*(1-s1)*(1-s1) -2s - 2s*s*s +4*s*s -
            3*s*s1*(s+ 3*s*s1*s1 -2*s*s1 -(s*s1)^2 -s1*s1)
        Dr - 9*s*s1*(1-s1)(1-s)(s-s1)

    v1x = (ce+b-e-bf)/(bd-ae);
    v2x = (d-a-cd+fa)/(bd-ae);
    for equations as -
    a*v1x + b*v2x + c = 1
    d*v1x + e*v2x + f = 1
    where,
    a = 3 + 3*s(1/3)^2 - 6*s(1/3)
    b = 3*(s(1/3) - s(1/3)^2)
    c = (s(1/3)^2)
    d = 3 + 3*s(2/3)^2 - 6*s(2/3)
    e = 3*(s(2/3) - s(2/3)^2)
    f = (s(2/3)^2)

*/
void computeInverseBezierMatrices(Spline &p){

    double s_1by3 = integrate(p, 0, 1.0/3);
    double s_2by3 = integrate(p, 0, 2.0/3);
    double s_1 = integrate(p, 0, 1);
    double sm_1by3 = s_1by3/s_1;
    double sm_2by3 = s_2by3/s_1;

    //std::cout << "Value of s and s1 " << s_1by3 << " " << s_2by3 << "" << s_1;
    double a = 9*(sm_1by3 + pow(sm_1by3,3) - 2*pow(sm_1by3,2));
    double b = 9*(pow(sm_1by3,2) - pow(sm_1by3,3));
    double c = 3*pow(sm_1by3,3);
    double d = 9*(sm_2by3 + pow(sm_2by3,3) - 2*pow(sm_2by3,2));
    double e = 9*(pow(sm_2by3,2) - pow(sm_2by3,3));
    double f = 3*pow(sm_2by3,3);

    double v1y = ((c*e)+(2*b)-e-(b*f))/((b*d)-(a*e));
    double v2y = (d-(2*a)-(c*d)+(f*a))/((b*d)-(a*e));

    //std::cout << v1y << " " << v2y << " " << b*d - a*e;
    double bm[] = {-1,3,-3,1,3,-6,3,0,-3,3,0,0,1,0,0,0};
    double vx[] = {0, sm_1by3, sm_2by3, 1}; //to be changed, but doesn't matter as of now
    double vy[] = {0, v1y, v2y, 1};

    gsl_matrix_view B = gsl_matrix_view_array(bm, 4,4);
    gsl_matrix_view Vx = gsl_matrix_view_array(vx, 4,1);
    gsl_matrix_view Vy = gsl_matrix_view_array(vy, 4,1);


    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &B.matrix, &Vx.matrix, 0,&IBVx.matrix);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &B.matrix, &Vy.matrix, 0,&IBVy.matrix);
}

double get_ufroms(double s){

    return invbvy[0]*s*s*s + invbvy[1]*s*s + invbvy[2]*s + invbvy[3];
}

double get_ufroms_sp(double s){

    double u,du,d2u;
    alglib::spline1ddiff(splineSU, s, u, du, d2u);
    if(u>0.999)return 1;
    else return u;
}

double getArcLengthParam(Spline& p, double s, double full, int *itr, int method) {
    // newton's method to find u for which arlength(p(0) to p(u)) = s;

  if (full < 0) {
    full = integrate(p, 0, 1);
  }

//  for(double i=0;i<=1;i+=0.001){
//      std::cout << p.xdd(i) << " " << p.ydd(i);
//  }

	if (s < 0) s = 0; //hard code
  assert(s >= 0);
  double u = s/full;  // initial guess;
  if(method==0)u = get_ufroms_sp(s);
  else if(method==1)u = get_ufroms(s/full);
  //return u;
  //printf("\n Value of u is %f and ini is %f", get_ufroms(s/full), s/full);

  double error = 1000;
  int iter = 0;
  while (fabs(error) > 1e-3 && iter < 60) {
    iter++;
    error = integrate(p,0,u)-s;
    u = u - error/p(u);
  }
  if (itr)
      *itr = iter;

  //if(iter>20 || my_code_time > 50000)
  //std::cout << "Time for optimisation, Iterations - " << my_code_time ;//<< " " << iter;
  //std::cout << "\n Value intial is " << s/full << ",after NR is " << u<<" bezApp is " << get_ufroms(s/full) << "SplineApp is " << get_ufroms_sp(s);

  return u;
}
}
