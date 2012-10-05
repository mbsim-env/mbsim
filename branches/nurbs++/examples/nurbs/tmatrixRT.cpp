#include <matrixRT.h>

int main(){
  using namespace PLib ; 
  Point3Df v(1,1,1) ;
  Point3Df w ;
  MatrixRT_FLOAT A ;

  cout << "v = " << v << endl ;; 
  A.rotate(M_PI/2.0,0,0) ;
  w = A*v ;
  cout << "rotate in x by 90 deg = " << w << endl ;
  A.rotate(0,M_PI/2.0,0) ;
  w = A*v ;
  cout << "rotate in y by 90 deg = " << w << endl ;
  A.rotate(0,0,M_PI/2.0) ;
  w = A*v ;
  cout << "rotate in z by 90 deg = " << w << endl ;
}
