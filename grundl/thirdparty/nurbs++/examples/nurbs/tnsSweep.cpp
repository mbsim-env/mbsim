#include <nurbsS.h>

int main(){
  using namespace PLib ; 
  Vector_Point3Df tp,cp ;
  PlNurbsCurvef T,C ;
  // setup the trajectory curve
  tp.resize(6) ;
  tp[0] = Point3Df(0,0,0) ;
  tp[1] = Point3Df(0,30,10) ;
  tp[2] = Point3Df(20,60,20) ;
  tp[3] = Point3Df(40,70,20) ;
  tp[4] = Point3Df(60,80,20) ;
  tp[5] = Point3Df(70,60,25) ;
  
  T.globalInterp(tp,3) ;

  // setup the curve to sweep

  cp.resize(4) ;
  cp[0] = Point3Df(-5,0,0) ;
  cp[1] = Point3Df(0,0,-5) ;
  cp[2] = Point3Df(3,0,10) ;
  cp[3] = Point3Df(5,0,0) ;

  C.globalInterp(cp,3) ;

  // generates a surface by sweeping

  PlNurbsSurfacef S ;

  S.sweep(T,C,5) ;
  S.writeVRML("tnsSweepA.wrl",redColor,20,50) ;


  C.makeCircle(Point3Df(0,0,0),Point3Df(1,0,0),Point3Df(0,0,1),5,0,1.5*M_PI) ;
  S.sweep(T,C,8) ;
  S.writeVRML("tnsSweepB.wrl",blueColor,20,50) ;

  S.writeRIB("test.rib",Color(30,255,200),Point3Df(0,1,0)) ;

   
  // create a scaling function

  PlNurbsCurvef scale ;

  Vector_Point3Df scaleP(3) ;

  scaleP[0] = Point3Df(1,1,1) ;
  scaleP[1] = Point3Df(0.5,0.5,2.0) ;
  scaleP[2] = Point3Df(1,1,1) ;

  scale.globalInterp(scaleP,2) ;

  S.sweep(T,C,scale,5,0) ;

  S.writeVRML("tnsSweepC.wrl",greenColor,20,50) ;


  S.sweep(T,C,scale,5,0) ;
  S.writeVRML("tnsSweepC.wrl",greenColor,20,50) ;

  S.sweep(T,C,5,1,0) ;
  S.writeVRML("tnsSweepD.wrl",greenColor,20,50) ;

  
  C.makeCircle(Point3Df(0,0,0),Point3Df(1,0,0),Point3Df(0,0,1),1,-M_PI/4.0,M_PI/3.0) ;
  S.makeFromRevolution(C,Point3Df(0,0,0),Point3Df(0,0,1)) ;
  S.writeVRML("tnsSweepE.wrl",Color(255,0,255),20,30) ;
  S.write("tnsSweepE.ns") ;

  cout << "The results of the sweep test can be viewed in:\n" <<
    "\t tnsSweepA.wrl\n\t tnsSweepB.wrl\n\t tnsSweepC.wrl\n\t tnsSweepD.wrl\n" ;
  cout << "In tnsSweepE.wrl, you'll see a surface of revolution\n" ;
  return 0 ;
}
