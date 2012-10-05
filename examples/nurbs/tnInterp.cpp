#include <nurbs.h>

int main(){
  using namespace PLib ; 
  int i ;
  cout << "Testing NURBS interpolation and approximation\n" ;

  Vector_Point3Df points(10) ;

  points[0] = Point3Df(20,20,0) ;
  points[1] = Point3Df(20,80,0) ;
  points[2] = Point3Df(20,120,0) ;
  points[3] = Point3Df(20,160,0) ;
  points[4] = Point3Df(80,200,0) ;
  points[5] = Point3Df(120,200,0) ;
  points[6] = Point3Df(160,160,0) ;
  points[7] = Point3Df(160,120,0) ;
  points[8] = Point3Df(120,80,0) ;
  points[9] = Point3Df(80,80,0) ;


  Vector_Point3Df deriv(10) ;
  deriv[0] = Point3Df(0,1,0) ;
  deriv[1] = Point3Df(0,1,0) ;
  deriv[2] = Point3Df(0,1,0) ;
  deriv[3] = Point3Df(0,1,0) ;
  deriv[4] = Point3Df(1,0,0) ;
  deriv[5] = Point3Df(1,0,0) ;
  deriv[6] = Point3Df(0,-1,0) ;
  deriv[7] = Point3Df(0,-1,0) ;
  deriv[8] = Point3Df(-1,0,0) ;
  deriv[9] = Point3Df(-1,0,0) ;

  PlNurbsCurvef curveA,curveB,curveC,curveD;

  curveA.globalInterp(points,3) ;
  curveB.leastSquares(points,3,5) ;
  curveC.leastSquares(points,3,8) ;
  curveD.globalInterpD(points,deriv,3,1) ;

  curveA.writePS("tnInterpA.ps",1,2.0) ;
  curveB.writePS("tnInterpB.ps",1,2.0) ;
  curveC.writePS("tnInterpC.ps",1,2.0) ;
  curveD.writePS("tnInterpD.ps",1,2.0) ;

  #ifdef WITH_IMAGE_MAGICK
  IM_ColorImage result ;
  
  result.resize(250,250) ;
  result.reset(Color(255,255,255)) ;


  cout << "The global interpolation result is represented in white.\n" ;
  curveA.drawImg(result,Color(255,255,255),0.001) ;
  cout << "The least squares approximation with 5 points is in cyan.\n" ;
  curveB.drawImg(result,Color(0,255,255),0.001) ;
  cout << "The least squares approximation with 8 points is in blue.\n" ;  
  curveC.drawImg(result,Color(0,0,255),0.001) ;
  cout << "The global interpolation with the 1st derivative specified is in yellow.\n" ;
  curveD.drawImg(result,Color(255,255,0),0.001) ;

  cout << "The points are in red.\n" ;
  for(i=0;i<points.n();++i){
    for(j=-2;j<3;++j){
      int a = (int)points[i].y() ;
      int b = (int)points[i].x() ;
      result(a+j,b) = Color(255,0,0) ;
      result(a,b+j) = Color(255,0,0) ;
    }
  }
  cout << "The result is in tnInterp.pnm\n" ;
  result.write("tnInterp.pnm");

  #endif

  cout << "Done with the interpolation and approximation test.\n" ;
}
