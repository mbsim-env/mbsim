#include <vector.h>

int main(){
  Vector_INT b(5) ;
  Vector_FLOAT c(5) ;

  b[0] = c[0] = 0.4 ;
  b[1] = c[1] = 8.9 ;
  b[2] = c[2] = 3.5 ;
  b[3] = c[3] = 0.2 ;
  b[4] = c[4] = 5 ;
  
  cout << "       C = " << c ;
  c.qSort() ;
  cout << "sorted C = " << c ; 
  cout << endl ;
  cout << "       B = " << b ;
  b.qSort() ;
  cout << "sorted B = " << b ; 

  /*
  NMatrix::Error error("tvqSort") ;

  cout << "still good ? " << error.opfx() << endl ;
  error << "This makes sure the warning system works.\n" ;
  error.fatal() ;
  */
  return 0 ;
}
