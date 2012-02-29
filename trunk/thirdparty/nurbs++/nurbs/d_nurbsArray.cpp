#include "nurbsArray.cpp"

namespace PLib {

#ifdef NO_IMPLICIT_TEMPLATES

  template class NurbsCurveArray<double,3> ;
  template class NurbsSurfaceArray<double,3> ;

  template class NurbsCurveArray<double,2> ;

#endif

}
