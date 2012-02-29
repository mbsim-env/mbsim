#include <nurbs.h>
#include <nurbsS.h>
#include <vector.h>

int main(){
  int deg = 3 ;

  using namespace PLib ; 

  Vector_HPoint3Df P(11) ;

  P[0] = HPoint3Df(0,000,0,1) ;
  P[1] = HPoint3Df(0,050,0,1) ;
  P[2] = HPoint3Df(0,100,0,1) ;
  P[3] = HPoint3Df(0,150,0,1) ;
  P[4] = HPoint3Df(0,200,0,1) ;
  P[5] = HPoint3Df(0,250,0,1) ;
  P[6] = HPoint3Df(0,300,0,1) ;
  P[7] = HPoint3Df(0,350,0,1) ;
  P[8] = HPoint3Df(0,400,0,1) ;
  P[9] = HPoint3Df(0,450,0,1) ;
  P[10]= HPoint3Df(0,500,0,1) ;
  
  PlNurbsCurvef curve1 ;

  curve1.globalInterpH(P,deg) ;
  double length = curve1.length(1e-6,100);
  cout << "Length = " << length << "\n" ;

  double radius = 10; 
  double area ;

  PlNurbsSurfacef surface ;

  PlNurbsCurvef curve2;
  curve2.makeCircle(Point3Df(0,0,0),Point3Df(1,0,0),Point3Df(0,0,1),radius,0,2*M_PI) ;
  surface.sweep(curve1,curve2,10) ;
  surface.writeDisplayQUADMESH("cylinder.obj");
  surface.writeVRML("cylinder.wrl");
  area = surface.area(1e-3,25);
  cerr << "Area cylinder = " << area 
       << " ( " << 500*2*M_PI*10
       << ")" << "\n" << std::flush ;
  
  surface.makeSphere(Point3Df(0,0,0),radius) ;

  surface.writeDisplayQUADMESH("sphere.obj");
  area = surface.area(1e-3,25);
  cerr << "Area sphere = " << area 
       << " ( " << (4*M_PI*radius*radius)
       << ")" << "\n" ;

  surface.writeVRML("sphere.wrl",Color(0,0,255));

  return 0 ; 
}
