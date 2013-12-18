/* test of intcc2.c */

#include <math.h>
#include <stdio.h>

#include "integrate.h"

using namespace PLib ; 

int nf = 0;


struct TestFcn : public ClassPO<double> {
  double operator()(double x)
    {
      nf++;
      return 4 / (1 + x * x);
    }
  int nf ; 
  TestFcn(): nf(0) {;}
};

main()
{
    double  i, err;
    TestFcn f ;
    

    // the casting is necessary with g++ and -fno_implicit_templates
    i = integrate((ClassPO<double>*)&f, 0.0, 1.0, 1.0e-15, 3200, err);
    printf("I= integral_[0,1] 4/(1+x^2) dx\n");
    printf("I= %lg\t err= %lg\t N= %d\n", i, err, f.nf);

}

