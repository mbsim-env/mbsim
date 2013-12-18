// The first functions are used to compute the bezierMatrix coefficients
// I'm keeping them here in case I need them.
// but this file is used to test the validity of the length function
// this is bases partly on equation 6.86 from the NURBS book

#include <nurbs.h>
#include <fstream>
#include <matrix.h>

using namespace PLib ; 

// Generates the bezier matrix coefficients
void bezierMatrix(int deg, PlMatrix_float& Mp){
  int i,j ;
  PlMatrix_float bin(deg+1,deg+1) ;

  binomialCoef(bin) ;
  Mp.resize(deg+1,deg+1) ;

  for(i=0;i<deg;++i) // set the upper matrix to 0
    for(j=i+1;j<=deg;++j)
      Mp(i,j) = 0 ;
  Mp(0,0) = Mp(deg,deg) = 1.0 ;

  if(deg % 2) 
    Mp(deg,0) = -1.0 ;
  else
    Mp(deg,0) = -1.0 ;
  float sign = -1.0 ;

  for(i=1;i<deg;++i){ // compute first column, last row and the diagonal
    Mp(i,i) = bin(deg,i) ;
    Mp(i,0) = Mp(deg,deg-i) = sign*Mp(i,i) ;
    sign = -sign ;
  }
  // compute remaining elements
  int k1 = (deg+1)/2 ; 
  int pk = deg-1 ;
  for(int k=1;k<k1;++k){
    sign = -1.0 ;
    for(j=k+1; j<=pk ; ++j){
      Mp(j,k) = Mp(pk,deg-j) = sign*bin(deg,k)*bin(deg-k,j-k) ;
      sign = -sign ;
    }
    pk = pk-1 ;
  }
}

inline float power(float x, int p){
  float r = x ;
  if(p==0)
    return 1 ;
  for(int i=1;i<p;++i)
    r *= x ;
  return r ;
}

// generates the reparametrization matrix
void reparMatrix(int deg, float c, float d, PlMatrix_float& R){
  PlMatrix_float bin(deg+1,deg+1) ;
  binomialCoef(bin) ;

  R.resize(deg+1,deg+1); 
  for(int i=0;i<R.rows();++i)
    for(int j=0;j<R.cols();++j){
      if(j<i)
	R(i,j) = 0 ;
      else
	R(i,j) = bin(j,i)*power(c,i)*power(d,j-i) ;
    }
}

// Generates the power coefficients
void findPowerCoef(const PlMatrix_float& Mp, const PlMatrix_float& R, const Vector_HPoint3Df& P, Vector_HPoint3Df& b){
  // [b] = R * Mp * P
  PlMatrix_float a(Mp.rows(),Mp.cols()) ;
  a = R*Mp ;
  b.resize(P.n()) ;

  for(int i=0;i<b.n();++i){
    b[i] = HPoint3Df(0,0,0,0) ;
    for(int j=0;j<P.n();++j)
      b[i] += a(i,j)*P[j] ;
  }
}

int main(){
  int i,j ;

  PlNurbsCurvef circle  ;

  circle.makeCircle(Point3Df(0,0,0),Point3Df(1,0,0),Point3Df(0,0,1),1,0,2*M_PI) ;

  cout << "Testing the length function.\n" ;
  cout << " You should note that I'm looping 500 times the computation for profiling purposes. Remove the loops if you want a faster result.\n" ; 

  cout << "A circle is represented with a NURBS curve<float> of degree 2.\n" ;
  float l;
  for(i=0;i<500;++i)
    l = circle.length() ;
  cout << "it's length is " << l << "\terror = " << absolute(2*M_PI-l) << endl ; 

  cout << "Now we use a degree 3 circle to make sure it's still valid\n" ;
  circle.degreeElevate(1) ;
  for(i=0;i<500;++i)
    l = circle.length() ;
  cout << "it's length is " << l << "\terror = " << absolute(2*M_PI-l) << endl ; 

  cout << "\nIn case you forgot, the value should be 2*PI\n" ; 


  cout << "\nNow we test if we can find the length of the curve for a\n"
    "range of the circle: [0.25,0.75].\n" ;
  for(i=0;i<500;++i)
    l = circle.lengthIn(0.25,0.75) ;
  cout << "The length is " << l << "\terror = " << absolute(M_PI-l) << endl ; 

  return 0 ;
}
