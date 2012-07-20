/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include "ansatz_functions.h"

using namespace fmatvec;

ansatz_function::ansatz_function(int n, double l): Ord(n), L(l) {
}

ansatz_function::~ansatz_function() {
}

SymMat ansatz_function::MatIntWWT() {
  SymMat retMat(Dim, INIT, 0);
  for(int j=0; j<Dim; j++)
    for(int i=0; i<Dim; i++)
      retMat(i, j)=Intwiwj(i, j);
  return retMat;
}

SymMat ansatz_function::MatIntWSWST() {
  SymMat retMat(Dim, INIT, 0);
  for(int j=0; j<Dim; j++)
    for(int i=0; i<Dim; i++)
      retMat(i, j)=Intwsiwsj(i, j);
  return retMat;
}

Vec ansatz_function::VecW0() {
  Vec retVec(Dim, INIT, 0);
  for(int i=0; i<Dim; i++)
    retVec(i)=wi0(i);
  return retVec;
}

Vec ansatz_function::VecWL() {
  Vec retVec(Dim, INIT, 0);
  for(int i=0; i<Dim; i++)
    retVec(i)=wiL(i);
  return retVec;
}

Vec ansatz_function::VecIntW() {
  Vec retVec(Dim, INIT, 0);
  for(int i=0; i<Dim; i++)
    retVec(i)=Intwi(i);
  return retVec;
}

Vec ansatz_function::VecW(double xi) {
  Vec retVec(Dim, INIT, 0);
  for(int i=0; i<Dim; i++)
    retVec(i)=wi(i, xi);
  return retVec;
}

Vec ansatz_function::VecWS(double xi) {
  Vec retVec(Dim, INIT, 0);
  for(int i=0; i<Dim; i++)
    retVec(i)=wis(i, xi);
  return retVec;
}

int ansatz_function::order() {
  return Ord;
}

int ansatz_function::dim() {
  return Dim;
}

// Approximate with the Galerkin-Methode using harmonic function

ansatz_function_harmonic::ansatz_function_harmonic(int n, double l) : ansatz_function(n, l) { 
  Dim=2*Ord+1; 
}

  double ansatz_function_harmonic::Intwiwj(int i, int j) {
    if(i==0 && j==0)
      return L;
    if(i==j)
      return L/2.;
    if((i+j%4)%4==1)
    {
      int k, l;
      if(i%2==0) { k=i/2; l=j/2+1; } else { l=i/2+1; k=j/2; }
      return 2./double(k+l)/double(l-k)/M_PI*l*L;
    }
    return 0;
  }

  double ansatz_function_harmonic::Intwsiwsj(int i, int j) {
    if(i==j)
    { int n=(i+1)/2; return n*n*M_PI*M_PI/2/L; }
    if(i>0 && j>0 && (i+j%4)%4==1) {
      int k, l;
      if(i%2==0) { k=i/2; l=j/2+1; } else { l=i/2+1; k=j/2; }
      return 2.*k*k*l*M_PI/(k+l)/(l-k)/L;
    }
    return 0;
  }

  double ansatz_function_harmonic::wi0(int i) {
    if(i%2==0)
      return 1;
    return 0;
  }

  double ansatz_function_harmonic::wiL(int i) {
    if(i%4==0)
      return 1;
    if(i%4==2)
      return -1;
    return 0;
  }

  double ansatz_function_harmonic::Intwi(int i) {
    if(i==0)
      return L;
    if(i%4==1)
      return 2./double((i+1)/2)*L/M_PI;
    return 0;
  }

  double ansatz_function_harmonic::wi(int i, double xi) {
    if(i%2==1)
      return sin(.5*double(i+1)*M_PI*xi);
    else if(i%2==0 && i!=0)
      return cos(.5*i*M_PI*xi);
    return 1;
  }

  double ansatz_function_harmonic::wis(int i, double xi) {
    if(i%2==1)
      return .5*double(i+1)*M_PI*cos(.5*double(i+1)*M_PI*xi);
    else if(i%2==0 && i!=0)
      return -.5*i*M_PI*sin(.5*i*M_PI*xi);
    return 0;
  }

// Approximate with the Galerkin-Methode using polynom function

ansatz_function_polynom::ansatz_function_polynom(int n, double l) : ansatz_function(n, l) {
  Dim=Ord+1;
}

double ansatz_function_polynom::Intwiwj(int i, int j) {
  return L/(i+j+1);
}

  double ansatz_function_polynom::Intwsiwsj(int i, int j) {
    if(i==0 || j==0)
      return 0;
    else
      return double(i*j)/L/(i+j-1);
  }

  double ansatz_function_polynom::wi0(int i) {
    if(i==0)
      return 1;
    else
      return 0;
  }

double ansatz_function_polynom::wiL(int i) {
  return 1;
}

double ansatz_function_polynom::Intwi(int i) {
  return L/(i+1);
}

double ansatz_function_polynom::wi(int i, double xi) {
  return pow(xi, i);
}

  double ansatz_function_polynom::wis(int i, double xi) {
    if(i==0)
      return 0;
    else
      return double(i)*pow(xi, i-1);
  }



// Approximate with the Galerkin-Methode using B-Spline functions (Order 3)

ansatz_function_BSplineOrd3::ansatz_function_BSplineOrd3(int n, double l) : ansatz_function(n, l) {
  Dim=Ord+2;
}

double ansatz_function_BSplineOrd3::Intwiwj(int i, int j) {
  if((i==0 && j==0) || (i==Dim-1 && j==Dim-1)) return L/Ord*1./20;
  if((i==1 && j==1) || (i==Dim-2 && j==Dim-2)) return L/Ord*1./2;
  if((i==0 && j==1) || (i==1 && j==0) || (i==Dim-1 && j==Dim-2) || (i==Dim-2 && j==Dim-1)) return L/Ord*13./120;
  if(abs(i-j)==0) return L/Ord*11./20;
  if(abs(i-j)==1) return L/Ord*13./60;
  if(abs(i-j)==2) return L/Ord*1./120;
  return 0;
}

double ansatz_function_BSplineOrd3::Intwsiwsj(int i, int j) {
  if((i==0 && j==0) || (i==Dim-1 && j==Dim-1)) return Ord/L*1./3;
  if((i==1 && j==1) || (i==Dim-2 && j==Dim-2)) return Ord/L*2./3;
  if((i==0 && j==1) || (i==1 && j==0) || (i==Dim-1 && j==Dim-2) || (i==Dim-2 && j==Dim-1)) return -Ord/L*1./6;
  if(abs(i-j)==0) return Ord/L*1.;
  if(abs(i-j)==1) return -Ord/L*1./3;
  if(abs(i-j)==2) return -Ord/L*1./6;
  return 0;
}

double ansatz_function_BSplineOrd3::wi0(int i) {
  if(i==0 || i==1) return 1./2;
  return 0;
}

double ansatz_function_BSplineOrd3::wiL(int i) {
  if(i==Dim-1 || i==Dim-2) return 1./2;
  return 0;
}

double ansatz_function_BSplineOrd3::Intwi(int i) {
  if(i==0 || i==Dim-1) return L/Ord*1./6;
  if(i==1 || i==Dim-2) return L/Ord*5./6;
  return L/Ord*1;
}

double ansatz_function_BSplineOrd3::wi(int i, double xi) {
  int n0=i-2;
  double x=xi*Ord;
  if(x<n0) return 0;
  else if (x<n0+1) return .5*(-x+n0)*(-x+n0);
  else if (x<n0+2) return 2.*x*n0+3.*x-x*x-n0*n0-3.*n0-1.5;
  else if (x<n0+3) return .5*(n0+3.-x)*(n0+3.-x);
  return 0;
}

double ansatz_function_BSplineOrd3::wis(int i, double xi) {
  int n0=i-2;
  double x=xi*Ord;
  if(x<n0) return 0;
  else if(x<n0+1) return x-n0;
  else if(x<n0+2) return 2*n0+3-2*x;
  else if(x<n0+3) return x-n0-3;
  return 0;
}


// Approximate with the Galerkin-Methode using B-Spline functions (Order 4)

ansatz_function_BSplineOrd4::ansatz_function_BSplineOrd4(int n, double l) : ansatz_function(n, l) {
  Dim=Ord+3;
}

double ansatz_function_BSplineOrd4::Intwiwj(int i, int j) {
  if((i==0 && j==0) || (i==Dim-1 && j==Dim-1)) return L/Ord*1./252;
  if((i==1 && j==1) || (i==Dim-2 && j==Dim-2)) return L/Ord*151./630;
  if((i==2 && j==2) || (i==Dim-3 && j==Dim-3)) return L/Ord*599./1260;
  if((i==0 && j==1) || (i==1 && j==0) || (i==Dim-1 && j==Dim-2) || (i==Dim-2 && j==Dim-1)) return L/Ord*43./1680;
  if((i==0 && j==2) || (i==2 && j==0) || (i==Dim-1 && j==Dim-3) || (i==Dim-3 && j==Dim-1)) return L/Ord*1./84;
  if((i==1 && j==2) || (i==2 && j==1) || (i==Dim-2 && j==Dim-3) || (i==Dim-3 && j==Dim-2)) return L/Ord*59./280;
  if(abs(i-j)==0) return L/Ord*151./315;
  if(abs(i-j)==1) return L/Ord*397./1680;
  if(abs(i-j)==2) return L/Ord*1./42;
  if(abs(i-j)==3) return L/Ord*1./5040;
  return 0;
}

double ansatz_function_BSplineOrd4::Intwsiwsj(int i, int j) {
  if((i==0 && j==0) || (i==Dim-1 && j==Dim-1)) return Ord/L*1./20;
  if((i==1 && j==1) || (i==Dim-2 && j==Dim-2)) return Ord/L*1./3;
  if((i==2 && j==2) || (i==Dim-3 && j==Dim-3)) return Ord/L*37./60;
  if((i==0 && j==1) || (i==1 && j==0) || (i==Dim-1 && j==Dim-2) || (i==Dim-2 && j==Dim-1)) return Ord/L*7./120;
  if((i==0 && j==2) || (i==2 && j==0) || (i==Dim-1 && j==Dim-3) || (i==Dim-3 && j==Dim-1)) return -Ord/L*1./10;
  if((i==1 && j==2) || (i==2 && j==1) || (i==Dim-2 && j==Dim-3) || (i==Dim-3 && j==Dim-2)) return -Ord/L*11./60;
  if(abs(i-j)==0) return Ord/L*2./3;
  if(abs(i-j)==1) return -Ord/L*1./8;
  if(abs(i-j)==2) return -Ord/L*1./5;
  if(abs(i-j)==3) return -Ord/L*1./120;
  return 0;
}

double ansatz_function_BSplineOrd4::wi0(int i) {
  if(i==0) return 1./6;
  if(i==1) return 2./3;
  if(i==2) return 1./6;
  return 0;
}

double ansatz_function_BSplineOrd4::wiL(int i) {
  if(i==Dim-1) return 1./6;
  if(i==Dim-2) return 2./3;
  if(i==Dim-3) return 1./6;
  return 0;
}

double ansatz_function_BSplineOrd4::Intwi(int i) {
  if(i==0 || i==Dim-1) return L/Ord*1./24;
  if(i==1 || i==Dim-2) return L/Ord*1./2;
  if(i==2 || i==Dim-3) return L/Ord*23./24;
  return L/Ord*1;
}

double ansatz_function_BSplineOrd4::wi(int i, double xi) {
  int n0=i-3;
  double x=xi*Ord;
  if(x<n0) return 0;
  else if (x<n0+1) return -pow(-x+n0, 3)/6.;
  else if (x<n0+2) return 1.5*x*x*n0+2.*x*x-.5*x*x*x-1.5*x*n0*n0-4.*x*n0-2.*x+.5*n0*n0*n0+2.*n0*n0+2.*n0+2./3.;
  else if (x<n0+3) return 1.5*x*n0*n0+8.*x*n0-1.5*x*x*n0+10.*x-4.*x*x+.5*x*x*x-.5*n0*n0*n0-4.*n0*n0-10.*n0-22./3.;
  else if (x<n0+4) return pow(n0+4.-x, 3)/6.;
  return 0;
}

double ansatz_function_BSplineOrd4::wis(int i, double xi) {
  int n0=i-3;
  double x=xi*Ord;
  if(x<n0) return 0;
  else if (x<n0+1) return .5*(-x+n0)*(-x+n0);
  else if (x<n0+2) return 3.*x*n0+4.*x-1.5*x*x-1.5*n0*n0-4.*n0-2.;
  else if (x<n0+3) return 1.5*n0*n0+8.*n0-3.*x*n0+10.-8.*x+1.5*x*x;
  else if (x<n0+4) return -.5*(n0+4.-1.*x)*(n0+4.-1.*x);
  return 0;
}
