#include <matrixMat.h>
#include <stdlib.h>

const double MaxRandom = 32768 ; // 2^15

using namespace PLib ; 

int main(){

  // generating a random matrix

  Matrix_DOUBLE m(6,6) ;
  int i,j ;
  for(i=0;i<m.rows();++i)
    for(j=0;j<m.cols();++j){
      m(i,j) = double(rand())/MaxRandom*100.0 ;
    }
  
  cout << "m = \n" << m << endl ;

  LUMatrix<double> LU ;
  Vector<int> pivot ;

  int e ;
  LU.decompose(m) ;
  Matrix<double> inv = LU.inverse() ;

  cout << "The LU factorization of m is\n\n" << LU << endl ;
  cout << "gives the inverse as\n\n" << LU.inverse() << endl ;
  cout << "Computing the error as m-inverse(inverse(m)) gives:\n\n" ;
  LU.decompose(inv) ;
  cout << m-LU.inverse() << endl ;
  
  cout << endl << endl;
  cout << "Trying to solve the following equations:\n" ;
  cout << "\t 3a + 9b + 4c -  d + 8e = 15\n" ;
  cout << "\t10a + 9b + 4c + 8d +15e = 115 \n" ;
  cout << "\t 4a + 7b + 5c + 3d + 2e = 54\n" ;
  cout << "\t 9a - 3b + 7c + 5d + 6e = 121\n"; 
  cout << "\t 7a +19b - 4c +40d + 3e = 282\n" ;
  cout << "\t -a +  b +11c + 2d +13e = 90\n" ;
  cout << "We write the equation as A X = B  (where X is the unknown matrix)\n";

  Matrix<double> A,B,X ;

  A.resize(6,5) ;

  double am[30] = {3, 9, 4, -1, 8, 10, 9, 4, 8, 15, 4, 7, 5, 3, 2,9,-3,7,5,6,7,19,-4,40,3,-1,1,11,2,13} ;
  for(i=0;i<6;++i)
    for(j=0;j<5;++j)
      A(i,j) = am[i*5+j] ;

  cout << "A =\n" << A ;
  
  B.resize(6,1) ;
  B(0,0) = 15 ;
  B(1,0) = 115 ;
  B(2,0) = 54 ;
  B(3,0) = 121 ;
  B(4,0) = 282 ;
  B(5,0) = 90 ;

  X.resize(5,1) ;

  SVDMatrix<double> svd ;

  if(!svd.decompose(A))
    cerr << "Found some singularities!\n" ;
  svd.solve(B,X) ;
  cout << "B=" << B << endl ;

  cout << "\nX =\n" << X ;
  cout << "The result should be 3, -2, 6, 8 and 1\n" ;

  //cout << "The error is X(computed)-X = " << X(0,0)-3.0 << ", " << X(1,0)+2.0 << ", " << X(2,0)-6.0 << ", " << X(3,0)-8.0 << ", " << X(4,0)-1.0 << endl ;
 
  cout << "The above result was obtained using SVD with an overdetermined set of equations.\n" ;
  cout << "Using only the first 5 elements, we can solve the system using\nLU decomposition.\n" ;

  cout << "B = " << B << endl ;
  A.resizeKeep(5,5) ;
  cout << "B(ok) = " << B << endl ;
  B.resizeKeep(5,1) ;
  cout << "B = " << B << endl ;
  LU.decompose(A) ;
  cout << "B2 = " << B << endl ;
  X = LU.inverse()*B ;
  cout << "B = " << B << endl ;

  cout << "test =\n" << A*LU.inverse() << endl ;
    
  cout << "And we obtain a new X equal to \n" << X << endl ;

  cout << "The new error is X(computed)-X = " << X(0,0)-3.0 << ", " << X(1,0)+2.0 << ", " << X(2,0)-6.0 << ", " << X(3,0)-8.0 << ", " << X(4,0)-1.0 << endl ;

  cout << "\nEnd of matrixMat testing.\n" ;

  return 0 ;
}
