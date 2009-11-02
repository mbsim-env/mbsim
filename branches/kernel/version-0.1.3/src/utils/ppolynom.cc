/* Copyright (C) 2004-2006  Robert Huber
 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

 *
 * Contact:
 *   rzander@users.berlios.de
 *
 */
#include<config.h>
#include "ppolynom.h"

namespace MBSim {

  void PPolynom::setXF(const Vec &x, const Vec &f, std::string InterpolationMethod)
  {
  	assert(x.size() == f.size());
  	
    if(InterpolationMethod == "csplinePer") {
      calculateSplinePeriodic(x,f);   
    }
    else if(InterpolationMethod == "csplineNat") { 
      calculateSplineNatural(x,f);   
    }
    else if(InterpolationMethod == "plinear") {
    	calculatePLinear(x,f);
    }
    else {
      cout << "ERROR (PPolynom::setXF): No valid method to calculate pp-form" << endl;
      exit(-1);
    }
    
    index = 0;
    nPoly = coefs.rows();
    order = coefs.cols()-1;
  }
  
  Vec PPolynom::operator()(double x)
  {
    if(x>breaks(nPoly)) {
      cout << "ERROR (PPolynom::operator()): x out of range! x= " << x << ", upper bound= " << breaks(nPoly) << endl;
      throw 54;
    }   
    if(x<breaks(0)) {
      cout << "ERROR (PPolynom::operator()): x out of range! x= " << x << ", lower bound= " << breaks(0) << endl;
      throw 54;
    }   
    if(!(x>breaks(index) && x<breaks(index+1))) { // saved index still OK? otherwise search
      index = 0;
      while (index < nPoly && breaks(index) <= x) index++;
      index--;
    }   
    double dx = x - breaks(index); // local coordinate
    double yi = coefs(index,0);
    for(int i=1;i<=order;i++) { // Horner scheme
      yi = yi*dx+coefs(index,i);
    }
    return Vec(1,INIT,yi);
  }

  Vec PPolynom::diff1(double x)
  {
    if(x>breaks(nPoly)) {
      cout << "ERROR (PPolynom::diff1): x out of range! x= "  << x << ", upper bound= " << breaks(nPoly) << endl;
      throw 54;
    }  
    if(x<breaks(0)) {
      cout << "ERROR (PPolynom::diff1): x out of range!   x= "  << x << " lower bound= " << breaks(0) << endl;
      throw 54;
    }  
    if(!(x>breaks(index) && x<breaks(index+1))) { // saved index still OK? otherwise search
      index = 0;
      while (index < nPoly && breaks(index) <= x) index++;
      index--;
    }    
    double dx = x - breaks(index);
    double yi = coefs(index,0)*order;
    for(int i=1;i<order;i++) {
      yi = yi*dx+coefs(index,i)*(order-i);
    }
    return Vec(1,INIT,yi);
  }

  Vec PPolynom::diff2(double x) {
    if(x>breaks(nPoly)){
      cout << "ERROR (PPolynom::diff2): x out of range!   x= "  << x << " upper bound= " << breaks(nPoly) << endl;
      throw 54;
    }
    
    if(x<breaks(0)) {
      cout << "ERROR (PPolynom::diff2): x out of range!   x= "  << x << " lower bound= " << breaks(0) << endl;
      throw 54;
    }
    
    if(!(x>breaks(index) && x<breaks(index+1))) { // saved index still OK? otherwise search
      index = 0;
      while ( index < nPoly && breaks(index) <= x) index++;
      index--;
    }
    
    double dx = x - breaks(index);
    double yi = coefs(index,0)*order*(order-1);
    for(int i=1;i<=(order-2);i++){
      yi = yi*dx+coefs(index,i)*(order-i)*(order-i-1);
    }
    return Vec(1,INIT,yi);
  }

  void PPolynom::calculateSplinePeriodic(const Vec &x, const Vec &f)
  {
 	double hi, hii;
    int i;
    int N = x.size();
    if(f(0)!=f(f.size()-1)) {
      cout << "ERROR (PPolynom::calculateSplinePeriodic): f(0)= " << f(0) << "!=" << f(f.size()-1) << " =f(end)" << endl;
      throw 50;
    }
    SqrMat C(N-1,N-1,INIT,0.0);
    Vec rs(N-1,INIT,0.0);

    // Matrix C and vector rs C*c=rs
    for(i=0; i<N-3;i++) {
      hi = x(i+1) - x(i);
      hii = x(i+2)-x(i+1);
      C(i,i) = hi;
      C(i,i+1) = 2*(hi+hii);
      C(i,i+2) = hii;
      rs(i) = 3*((f(i+2)-f(i+1))/hii - (f(i+1)-f(i))/hi);
    }
    
    // last but one row
    i = N-3;
    hi = x(i+1)-x(i);
    hii = x(i+2)-x(i+1);
    C(i,i) = hi;
    C(i,i+1)= 2*(hi+hii);
    C(i,0)= hii;
    rs(i) = 3*((f(i+2)-f(i+1))/hii - (f(i+1)-f(i))/hi);

    // last row
    double h1 = x(1)-x(0);
    double hN_1 = x(N-1)-x(N-2);
    C(N-2,0) = 2*(h1+hN_1);
    C(N-2,1) = h1;
    C(N-2,N-2)= hN_1;
    rs(N-2) = 3*((f(1)-f(0))/h1 - (f(0)-f(N-2))/hN_1);

    // solve C*c = rs -> TODO BETTER: RANK-1-MODIFICATION FOR LINEAR EFFORT (Simeon - Numerik 1)
    Vec c = slvLU(C,rs);
    Vec ctmp(N);
    ctmp(N-1) = c(0); // CN = c1
    ctmp(0,N-2)= c;

    // vector ordering of the further coefficients
    Vec d(N-1,INIT,0.0);
    Vec b(N-1,INIT,0.0);
    Vec a(N-1,INIT,0.0);
    
    for(i=0; i<N-1; i++) {
      hi = x(i+1)-x(i);  
      a(i) = f(i);
      d(i) = (ctmp(i+1) - ctmp(i) ) / 3 / hi;
      b(i) = (f(i+1)-f(i)) / hi - (ctmp(i+1) + 2*ctmp(i) ) / 3 * hi;
    }

    breaks.resize(N);
    coefs.resize(N-1,4);
    breaks = x;
    coefs.col(0) = d;
    coefs.col(1) = c;
    coefs.col(2) = b;
    coefs.col(3) = a;
  }

  void PPolynom::calculateSplineNatural(const Vec &x, const Vec &f)
  {
  	// first row
    int i=0;
    int N = x.size();
    SqrMat C(N-2,N-2,INIT,0.0);
    Vec rs(N-2,INIT,0.0);
    double hi = x(i+1)-x(i);
    double hii = x(i+2)-x(i+1);
    C(i,i) = 2*hi+2*hii;
    C(i,i+1) = hii;
    rs(i) = 3*(f(i+2)-f(i+1))/hii - 3*(f(i+1)-f(i))/hi;    

	// last row
    i = (N-3);
    hi = x(i+1)-x(i);
    hii = x(i+2)-x(i+1);
    C(i,i-1) = hi;
    C(i,i) = 2*hii + 2*hi;
    rs(i) = 3*(f(i+2)-f(i+1))/hii - 3*(f(i+1)-f(i))/hi;

    for(i=1;i<N-3;i++) { 
      hi = x(i+1)-x(i);
      hii = x(i+2)-x(i+1);
      C(i,i-1) = hi;
      C(i,i) = 2*(hi+hii);
      C(i,i+1) = hii;
      rs(i) = 3*(f(i+2)-f(i+1))/hii - 3*(f(i+1)-f(i))/hi;
    }

    // solve C*c = rs with C tridiagonal
    Mat C_rs(N-2,N-1,INIT,0.0);  
    C_rs(0,0,N-3,N-3) = C; // C_rs=[C rs] for Gauss in matrix
    C_rs.col(N-2) = rs;
    for(i=1; i<N-2; i++) C_rs.row(i) = C_rs.row(i) - C_rs.row(i-1)*C_rs(i,i-1)/C_rs(i-1,i-1); // C_rs -> upper triangular matrix C1 -> C1 rs1
    Vec rs1 = C_rs.col(N-2);
    Mat C1 = C_rs(0,0,N-3,N-3);
    Vec c(N-2,INIT,0.0);
    for(i=N-3;i>=0 ;i--) { // backward substitution
      double sum_ciCi = 0; 
      for(int ii=i+1; ii<=N-3; ii++) sum_ciCi = sum_ciCi + C1(i,ii)*c(ii);
      c(i)= (rs1(i) - sum_ciCi)/C1(i,i);
    }
    Vec ctmp(N,INIT,0.0);
    ctmp(1,N-2) = c; // c1=cN=0 natural splines c=[ 0; c; 0]

	// vector ordering of the further coefficients
    Vec d(N-1,INIT,0.0);
    Vec b(N-1,INIT,0.0);
    Vec a(N-1,INIT,0.0);

    for(i=0; i<N-1; i++) {
      hi = x(i+1)-x(i);  
      a(i) = f(i);
      d(i) = (ctmp(i+1) - ctmp(i) ) / 3 / hi;
      b(i) = (f(i+1)-f(i)) / hi - (ctmp(i+1) + 2*ctmp(i) ) / 3 * hi;
    }

    breaks.resize(N);
    coefs.resize(N-1,4);
    breaks = x;
    coefs.col(0) = d;
    coefs.col(1) = ctmp(0,N-2);
    coefs.col(2) = b;
    coefs.col(3) = a;
  }
  
  void PPolynom::calculatePLinear(const Vec &x, const Vec &f)
  {
  	int N = x.size(); // number of supporting points
  	
  	breaks.resize(N);
  	breaks = x;
  	
    coefs.resize(N-1,2); // N-1 intervals with linear polynomials
    for(int i=1;i<N;i++) {
      double m = (f(i)-f(i-1))/(x(i)-x(i-1)); // slope
      coefs(i-1,0) = m;
      coefs(i-1,1) = f(i-1);
    }
  }
  
  void MDPPolynom::setXF(const vector<Mat> &xf, std::string InterpolationMethod)
  {
  	M = xf.size();
  	 	
  	for(int i=0;i<M;i++) {
    	components.push_back(new PPolynom());
    	components[i]->setXF(static_cast<Vec>(trans(xf[i](0,0,0,xf[i].cols()-1))),static_cast<Vec>(trans(xf[i](1,0,1,xf[i].cols()-1))),InterpolationMethod);
	}
  }
  
  Vec MDPPolynom::operator()(double x)
  {
  	Vec y(M,INIT,0.);
    for(int i=0;i<M;i++) {
    	y(i) = ((*components[i])(x))(0);
    }
    return y;
  }
  
  Vec MDPPolynom::diff1(double x)
  {
  	Vec y(M,INIT,0.);
    for(int i=0;i<M;i++) {
    	y(i) = (components[i]->diff1(x))(0);
    }
    return y;
  }
  
  Vec MDPPolynom::diff2(double x)
  {
  	Vec y(M,INIT,0.);
    for(int i=0;i<M;i++) {
    	y(i) = (components[i]->diff2(x))(0);
    }
    return y;
  }
}
