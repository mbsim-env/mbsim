#include "nurbsArray.cpp"

namespace PLib { 

#ifdef NO_IMPLICIT_TEMPLATES

  template class NurbsCurveArray<float,3> ;
  template class NurbsSurfaceArray<float,3> ;
  
  template class NurbsCurveArray<float,2> ;

#endif

}
