#include <nurbsS.h>

int main(){
  using namespace PLib ; 
  int i,j ;

  // Let's generate a simple test surface

  Matrix_Point3Df pts(6,6) ;

  for(i=0;i<pts.rows();++i){
    for(j=0;j<pts.cols();++j){
      pts(i,j).x() = j*0.4 ;
      pts(i,j).y() = i*0.4 ;
      pts(i,j).z() = 0 ;
    }
  }

  PlNurbsSurfacef surf ;

  surf.globalInterp(pts,3,3) ;


  Vector_Point3Df D(2) ;
  Vector_FLOAT ur(2),vr(1) ;
  Vector_INT Dur(2),Duk(2),Dvr(2),Dvk(2) ;
  PLib::BasicArray<Coordinate> keepPts(4) ;

  D[0] = Point3Df(0,0,0.7) ;
  D[1] = Point3Df(0,3,0) ;
  ur[0] = 0.5 ;
  ur[1] = 0.6 ;
  vr[0] = 0.5 ;
  Dur[0] = 0 ;
  Dvr[0] = 0 ;
  Duk[0] = 0 ;
  Dvk[0] = 0 ;
  Dur[1] = 0 ;
  Dvr[1] = 0 ;
  Duk[1] = 0 ;
  Dvk[1] = 1 ;
  
  keepPts[0] = Coordinate(1,1) ;
  keepPts[1] = Coordinate(5,5) ;
  keepPts[2] = Coordinate(5,1) ;
  keepPts[3] = Coordinate(1,5) ;

  cout << "Point at (0.5,0.5) is " << surf.pointAt(0.5,0.5) << endl ;
  if(!surf.movePoint(ur,vr,D,Dur,Dvr,Duk,Dvk,keepPts)){
    cout << "Ill defined problem!\n" ;
  } ;

  cout << "after modification it is " << surf.pointAt(0.5,0.5) << endl ;

  surf.writeVRML("tnsMovePointA.wrl",Color(0,0,255)) ;

  surf.write("tngl.ns");


  PlNurbsSurfacef surf2(surf) ;
  PlNurbsSurfacef surf3(surf) ; 

  cout << "Before (0.3,0.6) = " << surf(0.3,0.6) << endl ;
  surf.movePoint(0.3,0.6,Point3Df(-0.1,-0.1,-0.1)) ;
  Vector_Point3Df d(1) ;
  d[0] = Point3Df(-0.1,-0.1,-0.1) ;
  Vector_FLOAT uk(1),vk(1) ;
  uk[0] = 0.3 ;
  vk[0] = 0.6 ;
  Vector_INT du(1),dv(1) ;
  du[0] = 0 ;
  dv[0] = 0 ;
  PLib::BasicArray<Coordinate> fix(4) ; 

  Vector_INT di(1),dj(1) ; 
  di[0] = 0 ; 
  dj[0] = 0 ; 

  fix[0] = Coordinate(2,2) ; 
  fix[1] = Coordinate(0,3) ; 
  fix[2] = Coordinate(1,4) ; 
  fix[3] = Coordinate(2,5) ; 

  surf2.movePoint(uk,vk,d,du,dv,di,dj,fix) ;

  cout << "After  (0.3,0.6) = " << surf(0.3,0.6) << endl ;
  cout << "After  (0.3,0.6) = " << surf2(0.3,0.6) << endl ;
  

  cout << "surf P =\n" << surf.ctrlPnts()-surf3.ctrlPnts() << endl << endl ; 
  cout << "surf2 P =\n" << surf2.ctrlPnts()-surf3.ctrlPnts() << endl << endl ; 


  surf.write("tnglB.ns") ;

  surf2.writeVRML("tnsMovePointB.wrl",Color(0,0,255)) ;
  cout << "You can view the before/after view of the surface in\n" ;
  cout << "\ttnsMovePointA.wrl and tnsMovePointB.wrl\n" ;
}
