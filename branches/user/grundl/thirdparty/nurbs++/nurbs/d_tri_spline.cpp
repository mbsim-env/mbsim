#include "tri_spline.cpp"

namespace PLib {


#ifdef NO_IMPLICIT_TEMPLATES
  template class TriangularBSpline<double,3> ;
  template class RTriangularBSpline<double,3> ;

  template void convert(const NurbsSurface<double,3>& surf, RTriangularBSpline<double,3> &t1, RTriangularBSpline<double,3> &t2);

#endif

}
