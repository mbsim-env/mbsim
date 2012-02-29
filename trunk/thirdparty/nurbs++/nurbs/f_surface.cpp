#include "surface.cpp"

namespace PLib {

  template <>
    int ParaSurface<float,2>::intersectWith(const ParaSurface<float,2> &S, Point_nD<float,2>& p, float& u, float& v, float& s, float& t, int maxI, float um, float uM, float vm, float vM) const { 
    cerr << "NOT DEFINED FOR 2D SURFACES.\n";
    return 0;
  }
  
  template <>
    int ParaSurface<float,2>::intersectWith(const ParaSurface<float,2> &S, struct InterPoint<float,2> &iter, int maxI, float um, float uM, float vm, float vM) const {
    cerr << "NOT DEFINED FOR 2D SURFACES.\n";
    return 0;
  }
  
  template <>
    int ParaSurface<float,2>::writeVRML97(ostream &fout,const Color& color,int Nu,int Nv, float uS, float uE, float vS, float vE) const{
    cerr << "NOT DEFINED FOR 2D SURFACES.\n" ; 
    return 0;
  }
  
#ifdef NO_IMPLICIT_TEMPLATES
  
  template class InterPoint<float,2> ;
  template class InterPoint<float,3> ;
  
  template class BasicList<InterPoint<float,2> > ; 
  template class BasicList<InterPoint<float,3> > ; 
  
  template class ParaSurface<float,2> ;
  template class ParaSurface<float,3> ;
  
  template void intersectSurfaces(const ParaSurface<float,2>&, const ParaSurface<float,2>&, BasicList<InterPoint<float,2> >&, int, float, float, float, float) ;
  
  template void intersectSurfaces(const ParaSurface<float,3>&, const ParaSurface<float,3>&, BasicList<InterPoint<float,3> >&, int, float, float, float, float) ;
  
#endif 

}
