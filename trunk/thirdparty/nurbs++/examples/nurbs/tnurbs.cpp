#include <nurbs.h>

// testing of interpolation and approximation

int main(){
  int i ;
  // Initialize a NURBS curve and generate a list of points from it
  int deg = 3 ;
  Vector_HPoint3Df P(10) ;
  
  using namespace PLib ; 

  P[0] = HPoint3Df(30,30,0,1) ;
  P[1] = HPoint3Df(40,60,0,1) ;
  P[2] = HPoint3Df(80,60,0,1) ;
  P[3] = HPoint3Df(60,160,0,1) ;
  P[4] = HPoint3Df(100,80,0,1) ;
  P[5] = HPoint3Df(120,50,0,1) ;
  P[6] = HPoint3Df(120,50,0,1) ;
  P[7] = HPoint3Df(120,50,0,1) ;
  P[8] = HPoint3Df(160,90,0,1) ;
  P[9] = HPoint3Df(200,200,0,1) ;

  Vector_FLOAT U(10+deg+1) ;
  for(i=0;i<deg+1;++i)
    U[i] = 0 ;
  for(i=deg+1;i<P.n();++i)
    U[i] = float(i-deg)/float(P.n()-deg) ;
  for(i=P.n();i<U.n();++i)
    U[i] = 1.0 ;
  
  PlNurbsCurvef testCurve(P,U,deg) ;

  Vector_Point3Df Pts(100) ;

  for(i=0;i<Pts.n();++i){
    Pts[i] = testCurve.pointAt(float(i)/float(Pts.n()-1)) ;
  }

#ifdef WITH_IMAGE_MAGICK
  IM_ColorImage tstImage ;
  tstImage.resize(256,256) ;

  tstImage.reset(Color(255,255,255)) ;

  for(i=0;i<Pts.n();++i){
    tstImage(int(Pts[i].y()),int(Pts[i].x())) = Color(0,0,0) ;
  }


  tstImage.write("tnurbs.gif") ;
  cout << "The result can be viewed in tnurbs.gif\n" ;
#endif

  cout << "Testing the read/write interface \n" ;
  testCurve.write("tnurbs.nc");
  testCurve.read("tnurbs.nc");

  testCurve.writeVRML("tnurbs.wrl",2,18,blueColor,6,30) ;
  testCurve.writeVRML97("tnurbs97.wrl",2,18,blueColor,6,30) ;

  cout << "Testing the derivative computation\n" ;
  PlVector_HPoint3Df D ;

  testCurve.deriveAtH(0.5,1,D) ; 
  cout << "First derivative at 0.5 = \n\t" << D[1] ; 

  HPoint3Df d1 ; 
  d1 = testCurve.firstD(0.5) ; 

  cout << "\n\t" << d1 << endl ; 

  cout << "or in 3D =\n\t" ;

  PlVector_Point3Df D3 ;
  testCurve.deriveAt(0.5,1,D3) ;
  cout << D3[1] << endl ; 
  cout << "\t" << testCurve.firstDn(0.5) << endl ; 

  cout << "The result can be viewed in tnurbs.wrl (VRML 1.0)\n" ;
  cout << "and tnurbs97.wrl (VRML 2.0).\n" ;
  return 0 ;
}
