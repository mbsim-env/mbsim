#include <nurbs.h>

int main(){
  using namespace PLib ; 
  Vector_Point3Df pts(6) ;

  pts[0] = Point3Df(40,0,0) ;
  pts[1] = Point3Df(60,0,0) ;
  pts[2] = Point3Df(60,60,0) ;
  pts[3] = Point3Df(100,60,0) ;
  pts[4] = Point3Df(100,0,0) ;
  pts[5] = Point3Df(120,0,0) ;


  PlNurbsCurvef curve,tcurve ;

  curve.globalInterp(pts,3) ;
  tcurve = curve ;

  curve.writePS("tnMovePointA.ps",1,1.0) ;
  cout << "Point at 0.5 is " << curve.pointAt(0.5) << endl ;
  
  Vector_Point3Df D(1) ;
  Vector_INT dr(1) ;
  Vector_INT dk(1) ;
  Vector_FLOAT ur(1) ;
  Vector_INT fixCP(1) ;

  D[0] = Point3Df(0,-10,0) ;
  
  curve.movePoint(0.5,D) ;
  curve.writePS("tnMovePointB.ps",1,1.0) ;

  cout << "after moving by (0,-10,0) it is " << curve.pointAt(0.5) << endl ;

  D[0] = Point3Df(0,-10,0) ;
  dr[0] = 0 ;
  dk[0] = 0 ;
  ur[0] = 0.5 ;
  fixCP[0] = 2 ;

  curve = tcurve ;
  curve.movePoint(ur,D,dr,dk,fixCP) ;
  curve.writePS("tnMovePointC.ps",1,1.0) ;
  cout << "after moving by (0,-10,0) and fixing control point 2, it is " << curve.pointAt(0.5) << endl ;
  
  curve = tcurve ;

  D.resize(3) ;
  dr.resize(3) ;
  dk.resize(3) ;
  ur.resize(3) ;
  ur[0] = 0.4 ;
  ur[1] = 0.5 ;
  ur[2] = 0.6 ;
  D[0] = Point3Df(0,0,0) ;
  D[1] = Point3Df(0,-10,0) ;
  D[2] = Point3Df(0,0,0) ;
  dr[0] = 0 ;
  dr[1] = 1 ;
  dr[2] = 2 ;
  dk[0] = 0 ;
  dk[1] = 0 ;
  dk[2] = 0 ;
  
  curve.movePoint(ur,D,dr,dk) ;
  cout << "moving at 0.5 by (0,-10,0) but fixing both points at 0.4 and at 0.6" << endl ;
  curve.writePS("tnMovePointD.ps",1,1.0) ;
  

  Point3Df d0 = curve.derive3D(0.0,1) ;
  Point3Df d1 = curve.derive3D(1.0,1) ;

  cout << "The end derivatives are " << curve.derive3D(0,1) << " and " << 
    curve.derive3D(1.0,1) << endl ;

  cout << "Changing the end derivatives so that they're on the y-axis\n" ;
  cout << "\tand that the end points do not move.\n" ;
  D.resize(4) ;
  dr.resize(4) ;
  dk.resize(4) ;
  ur.resize(2) ;

  ur[0] = 0.0 ;
  ur[1] = 1.0 ;
  D[0] = D[1] = Point3Df(0,0,0) ;
  D[2] = Point3Df(-d0.x(),0,0) ;
  D[3] = Point3Df(-d1.x(),0,0) ;
  dr[0] = 0 ;
  dr[1] = 1 ;
  dr[2] = 0 ;
  dr[3] = 1 ;
  dk[0] = dk[1] = 0 ;
  dk[2] = dk[3] = 1 ;

  curve.movePoint(ur,D,dr,dk) ;
  cout << "Now the end derivatives are " << curve.derive3D(0,1) << " and " << 
    curve.derive3D(1.0,1) << endl ;
  curve.writePS("tnMovePointE.ps",1,1.0) ;


  curve = tcurve ;

  cout << "You can also move the tangents to be along a user defined vector.\n" ;
  cout << " For instance, to <-1,1,0> and <-1,-1,0>.\n" ;
  curve.setTangentAtEnd(Point_nD<float,3>(-1,1,0),Point_nD<float,3>(-1,-1,0));
  curve.writePS("tnMovePointF.ps",1,1.0) ;

  cout << "\nFinished with the movePoint test, the results for each step\n" ;
  cout << "can be viewed in tnMovePoint[ABCDEF].ps\n" ;

  return 0 ;
}
