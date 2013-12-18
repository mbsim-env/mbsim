#include "tri_spline.cpp"

namespace PLib {


#ifdef NO_IMPLICIT_TEMPLATES

  template class TriangularBSpline<float,3> ;
  template class RTriangularBSpline<float,3> ;

  template void convert(const NurbsSurface<float,3>& surf, RTriangularBSpline<float,3> &t1, RTriangularBSpline<float,3> &t2);

#endif

}
