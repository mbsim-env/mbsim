#include <nurbs.h>

int main(){
  using namespace PLib ; 

  #ifdef WITH_IMAGE_MAGICK
  
  cerr << "Testing the anti-aliased drawing of a nurbs curve\n" ;

  PlNurbsCurvef profile ;

  profile.makeCircle(Point3Df(0,0,0),Point3Df(1,0,0),Point3Df(0,0,1),5,0,M_PI) ;


  IM_ColorImage img ;

  img.resize(200,200) ;

  PlNurbsCurvef trajectory ;

  trajectory.makeCircle(Point3Df(100,100,0),70,M_PI*0.25,M_PI*0.75) ;

  trajectory.drawAaImg(img,Color(255,255,255),profile,3) ;

  Vector_HPoint3Df t2(4) ;
  
  t2[0] = HPoint3Df(40,50,0,1) ;
  //t2[1] = HPoint3Df(70,30,0) ;
  t2[1] = HPoint3Df(80,60,0,1) ;
  t2[2] = HPoint3Df(60,80,0,1) ;
  t2[3] = HPoint3Df(140,150,0,1) ;
  
  Vector_FLOAT U(8) ;
  U[0] = U[1] = U[2] = U[3] = 0.0 ;
  U[4] = U[5] = U[6] = U[7] = 1.0 ;

  PlNurbsCurvef trajectory2(t2,U,3) ;
  

  Vector_Point3Df pp(5) ;

  pp[0] = Point3Df(-7,0,0) ;
  pp[1] = Point3Df(-3,0,1) ;
  pp[2] = Point3Df(0,0,0) ;
  pp[3] = Point3Df(3,0,1) ;
  pp[4] = Point3Df(7,0,0) ;

  profile.globalInterp(pp,3) ;

  trajectory2.drawAaImg(img,Color(255,255,0),profile,3) ;

  trajectory2.makeCircle(Point3Df(80,80,0),50,0,M_PI) ;
  trajectory2.drawAaImg(img,Color(0,255,0)) ;

  img.write("tdrawAa.pnm") ;

  cerr << "The result is stored in tdrawAa.pnm\n" ;

  #else
  cerr << "This program require Image Magick\n" ;
  #endif

  return 0 ;
}
