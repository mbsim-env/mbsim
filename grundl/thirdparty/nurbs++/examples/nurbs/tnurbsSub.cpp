#include "nurbsSub.h"

namespace PLib {

template <class T, int N>
NurbsSurface<T,N>* generateTorus(T majorRadius, T minorRadius)
{
  // These define the shape of a unit torus centered about the origin. 
  T xvalues[] = { 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0, 1.0, 0.0 };
  T yvalues[] = { 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0 };
  T zvalues[] = { 0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0 };
  T offsets[] = { -1.0, -1.0, 0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0 };

  // Piecewise Bezier knot vector for a quadratic curve with four segments 
  T knotsMem[] = { 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4 };
  Vector<T> knots(knotsMem,12) ;

  int i, j;

  double r2over2 = sqrt( 2.0 ) / 2.0;
  double weight;

  NurbsSurface<T,N> *torus = new NurbsSurface<T,N> ;

  torus->resize(9,9,2,2) ;

  for (i = 0; i < 9; i++){
    for (j = 0; j < 9; j++) {
      HPoint_nD<T,N> hp ;
      weight = ((j & 1) ? r2over2 : 1.0) * ((i & 1) ? r2over2 : 1.0);
      // Notice how the weights are pre-multiplied with the x, y and z values
      hp.x() = xvalues[j]* (majorRadius + offsets[i] * minorRadius) * weight;
      hp.y() = yvalues[j]* (majorRadius + offsets[i] * minorRadius) * weight;
      hp.z() = (zvalues[i] * minorRadius) * weight;
      hp.w() = weight;
      torus->modCP(i,j,hp) ;
    }
  }

  // The knot vectors define piecewise Bezier segments 
  // (the same in both U and V).
  
  torus->modKnotU(knots) ;
  torus->modKnotV(knots) ;

  return torus;
}

template NurbsSurface<float,3>* generateTorus<float,3>(float majorRadius, float minorRadius);

}

main()
{
  using namespace PLib ; 
  NurbsSurface<float,3> *torus;

  float tolerance = 2.0;

  torus = generateTorus<float,3>( 1.3, 0.3 );
  
  NurbsSubSurface<float> surf(*torus) ;
  
  surf.drawSubdivisionPS("tnurbsSub.ps", tolerance );
  surf.drawSubdivisionVRML("tnurbsSub.wrl",0.1) ;
  surf.drawSubdivisionVRML97("tnurbsSub97.wrl",0.1) ;
}
