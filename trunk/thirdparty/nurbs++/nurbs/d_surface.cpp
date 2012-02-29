#include "surface.cpp"

namespace PLib {

  template <>
    int ParaSurface<double,2>::intersectWith(const ParaSurface<double,2> &S, Point_nD<double,2>& p, double& u, double& v, double& s, double& t, int maxI, double um, double uM, double vm, double vM) const {
    cerr << "NOT DEFINED FOR 2D SURFACES.\n"; 
    return 0;
  }
  
  template <>
    int ParaSurface<double,2>::intersectWith(const ParaSurface<double,2> &S, struct InterPoint<double,2> &iter, int maxI, double um, double uM, double vm, double vM) const {
    cerr << "NOT DEFINED FOR 2D SURFACES.\n"; 
    return 0;
  }
  
  template <>
    int ParaSurface<double,2>::writeVRML97(ostream &fout,const Color& color,int Nu,int Nv, double uS, double uE, double vS, double vE) const{
    cerr << "NOT DEFINED FOR 2D SURFACES.\n" ; 
    return 0;
  }

#ifdef NO_IMPLICIT_TEMPLATES
  
  template class InterPoint<double,2> ;
  template class InterPoint<double,3> ;
  
  template class BasicList<InterPoint<double,2> > ; 
  template class BasicList<InterPoint<double,3> > ; 
  
  template class ParaSurface<double,2> ;
  template class ParaSurface<double,3> ;
  
  template void intersectSurfaces(const ParaSurface<double,2>&, const ParaSurface<double,2>&, BasicList<InterPoint<double,2> >&, int, double, double, double, double) ;
  template void intersectSurfaces(const ParaSurface<double,3>&, const ParaSurface<double,3>&, BasicList<InterPoint<double,3> >&, int, double, double, double, double) ;
  
#endif 

}
