#include <nurbsS.h>
#include <nurbsSub.h>
#include <math.h>

int main(){
  Matrix_Point3Df Pts(4,5) ;
  int i,j ;

  using namespace PLib ; 


  for(i=0;i<Pts.rows();++i){
    for(j=0;j<Pts.cols();++j){
      Pts(i,j) = Point3Df(i,j,j) ;
    }
  }

  PlNurbsSurfacef surf ;

  surf.globalInterp(Pts,3,3) ;

  cerr << "Point at (0.5,0.5) = " << surf(0.5,0.5) << endl ;
  cerr << "\t=  " << surf.pointAt(0.5,0.5) << endl ;
  cerr << "\t normal = " << surf.normal(0.5,0.5) << endl ;

  for(i=0;i<Pts.rows();++i)
    for(j=0;j<Pts.cols();++j)
      Pts(i,j) = Point3Df(i*20,j*20,(1+cos(i+j))*20) ;

  surf.globalInterp(Pts,3,3) ;

  cerr << "Point at (0.5,0.5) = " << surf(0.5,0.5) << endl ;
  cerr << "\t=  " << surf.pointAt(0.5,0.5) << endl ;
  cerr << "\t normal = " << surf.normal(0.5,0.5) << endl ;

  surf.write("tnurbsS.ns") ;
  surf.read("tnurbsS.ns");
  surf.writeVRML("tnurbsS.wrl") ;
  surf.writePS("tnurbsS.ps",5,5,Point3Df(10,10,10),Point3Df(0,0,0)) ;
  

  PLib::NurbsSubSurface<float> sub(surf) ;
  sub.drawSubdivisionVRML("tnurbsSb.wrl",0.5);

  surf.writeVRML("tnurbsS2.wrl",Color(255,255,0),float(0.1)) ;


  NurbsCurvef curve ;
  curve.makeCircle(Point3Df(0.5,0.5,0),0.3);


  Vector_Point3Df pnt(100) ;

  for(i=0;i<100;++i){
    Point3Df param = curve.pointAt(float(i)/99.0) ;
    pnt[i] = surf.pointAt(param.x(),param.y());
  }

  NurbsCurvef curve2 ;

  curve2.globalInterp(pnt,3) ; 

  ofstream fout ;

  fout.open("tnurbsST.wrl");
  surf.writeVRML(fout,Color(255,255,255)) ;

  curve2.writeVRML(fout,0.1,30,Color(255,0,0),6,60) ;

  curve.writeVRML(fout,0.1,30,Color(0,255,0),6,60) ;

  curve.degreeElevate(7) ;

  for(i=0;i<curve.ctrlPnts().n();++i){
    HPoint3Df p = surf(curve.ctrlPnts(i).x(),curve.ctrlPnts(i).y()) ;
    curve.modCP(i,p) ;
  }

  curve.writeVRML(fout,0.1,30,Color(0,0,255),6,60);

  fout.close();


  //surf.writePOVRAY(0.1f,"tnurbsS.pov",Color(255,255,0),Point3Df(0,1,0),Point3Df(0,0,1)) ;

  cerr << "Testing the normal computation at the corner points\n" ;
  cerr << "normal(0,0) = " << surf.normal(0,0).unitLength() << endl ;
  cerr << "normal(0,1) = " << surf.normal(0,1).unitLength() << endl ;
  cerr << "normal(1,0) = " << surf.normal(1,0).unitLength() << endl ;
  cerr << "normal(1,1) = " << surf.normal(1,1).unitLength() << endl ;

  surf.makeTorus(Point3Df(0,0,0),4,1) ;
  surf.writeVRML("torus.wrl");

  return 0 ;
}
