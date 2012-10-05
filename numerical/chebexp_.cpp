#include "chebexp.cpp"

namespace PLib {

#ifdef NO_IMPLICIT_TEMPLATES

  template void chebexp(double (*f)(float), float a, float b, float eps, BasicArray<float> &c, float &err) ;
  template void chebexp(double (*f)(float,void*), void*, float a, float b, float eps, BasicArray<float> &c, float &err) ;
  template float chebeval(float x, const BasicArray<float> &c) ;
  
  template void chebexp(double (*f)(double), double a, double b, double eps, BasicArray<double> &c, double &err) ;
  template void chebexp(double (*f)(double,void*), void*, double a, double b, double eps, BasicArray<double> &c, double &err) ;
  template double chebeval(double x, const BasicArray<double> &c) ;
  
#endif  

}
