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

void refreshMatrix(){
}

alglib::spline1dinterpolant splineSU;
void computeSplineApprox(Spline &p){

    using namespace alglib;
    double n = 10; // number of points to interpolate on
    vector<double> s(n,0), u(n,0);

    for(int i=0;i<n;i++){
        u[i] = i*1.0/n;
        s[i] = integrate(p,0,u[i]);
    }
	
    alglib::real_1d_array AU, AS;
    AS.setcontent(s.size(), &(s[0]));
    AU.setcontent(u.size(), &(u[0]));
    //spline1dbuildcubic(AS, AU, u.size(), 1, ut2, 1, ut1, splineSU);
    spline1dbuildcubic(AS, AU, splineSU);
}

double get_ufroms_sp(double s){

    double u,du,d2u;
    alglib::spline1ddiff(splineSU, s, u, du, d2u);
    if(u>0.999)return 1;
    else return u;
}

double getArcLengthParam(Spline& p, double s, double full, int *itr) {
    // newton's method to find u for which arlength(p(0) to p(u)) = s;

  if (full < 0) {
    full = integrate(p, 0, 1);
  }

  if (s < 0) s = 0; //hard code
  assert(s >= 0);
  double u = get_ufroms_sp(s);

  double error = 1000;
  int iter = 0;
  while (fabs(error) > 1e-3 && iter < 60) {
    iter++;
    error = integrate(p,0,u)-s;
    u = u - error/p(u);
  }
  if (itr)
      *itr = iter;
  return u;
}
}
