#include <math.h>
#include <stdio.h>

#include "integrate.h"

using namespace PLib ; 

// But here goes,  (I = Integral sign)
// A = II F(S(u,v) du dv 
//   = I G(u) du
// where
//   G(u) = I F(s(u,v)) dv
// so compute A by calling
//   integrate(G,...) ;

struct F : public ClassPOvoid<double> {
  double operator()(double u, void* v)   {
      nf++;
      return exp(*((double*)v)) *  sin(u);
    }
  int nf ; 
  F(): nf(0) {;}
};

struct G : public ClassPO<double> {
  double operator()(double v)    {
      F f ;
      double err, integral;
      integral = integrate2((ClassPOvoid<double>*)&f,(double*) &v, 0.0,M_PI/2, 1.0e-15, 3200, err);
      ng += f.nf;
      return integral; 
    }
  int ng ; 
  G(): ng(0) {;}
};


main(){
    double  i, err;
    G g ;
    F f ;
    
    // the casting is necessary with g++ and -fno_implicit_templates
    i = integrate((ClassPO<double>*)&g, 0.0, 1.0, 1.0e-15,3200, err);
    printf("II= integral_[0,1] integral_[0,Pi/2] e^y sin(x)dy dx\n");
    printf("II= %lg\t err= %lg\t N= %d\n", i, err, g.ng);
    printf("The true value is II = (e-1) = %lg\n",(exp(1)-1));

    double a = 1;
    i = integrate2((ClassPOvoid<double>*)&f,(double*) &a,0.0, M_PI/2, 1.0e-15, 3200, err);
    printf("I= integral_[0,Pi/2] e sin(x) dx\n");
    printf("I= %lg\t err= %lg\t N= %d\n", i, err, f.nf);
    printf("The true value is I = e = %lg \n",(exp(1)));


}
