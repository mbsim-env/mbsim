/* Copyright (C) 2004-2009 MBSim Development Team
 *
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

#include <config.h>

#include "mbsim/utils/ppolynom.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"
#include "mbsim/mbsim_event.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {
      
  void PPolynom::setXF(const Vec &x, const Mat &f, std::string InterpolationMethod) {
    assert(x.size() == f.rows());

    if(InterpolationMethod == "csplinePer") {
      calculateSplinePeriodic(x,f);   
    }
    else if(InterpolationMethod == "csplineNat") { 
      calculateSplineNatural(x,f);   
    }
    else if(InterpolationMethod == "plinear") {
      calculatePLinear(x,f);
    }
    else throw MBSimError("ERROR (PPolynom::setXF): No valid method to calculate pp-form");

    index = 0;
    nPoly = x.size()-1;
    order = coefs.size()-1;
  }

  void PPolynom::calculateSplinePeriodic(const Vec &x, const Mat &f) {
    double hi, hii;
    int N = x.size();
    if(nrm2(f.row(0)-f.row(f.rows()-1))>epsroot()) throw MBSimError("ERROR (PPolynom::calculateSplinePeriodic): f(0)= "+numtostr(f.row(0))+"!="+numtostr(f.row(f.rows()-1))+" =f(end)");
    SqrMat C(N-1,N-1,INIT,0.0);
    Mat rs(N-1,f.cols(),INIT,0.0);

    // Matrix C and vector rs C*c=rs
    for(int i=0; i<N-3;i++) {
      hi = x(i+1) - x(i);
      hii = x(i+2)-x(i+1);
      C(i,i) = hi;
      C(i,i+1) = 2*(hi+hii);
      C(i,i+2) = hii;
      rs.row(i) = 3*((f.row(i+2)-f.row(i+1))/hii - (f.row(i+1)-f.row(i))/hi);
    }

    // last but one row
    hi = x(N-2)-x(N-3);
    hii = x(N-1)-x(N-2);
    C(N-3,N-3) = hi;
    C(N-3,N-2)= 2*(hi+hii);
    C(N-3,0)= hii;
    rs.row(N-3) = 3*((f.row(N-1)-f.row(N-2))/hii - (f.row(N-2)-f.row(N-3))/hi);

    // last row
    double h1 = x(1)-x(0);
    double hN_1 = x(N-1)-x(N-2);
    C(N-2,0) = 2*(h1+hN_1);
    C(N-2,1) = h1;
    C(N-2,N-2)= hN_1;
    rs.row(N-2) = 3*((f.row(1)-f.row(0))/h1 - (f.row(0)-f.row(N-2))/hN_1);

    // solve C*c = rs -> TODO BETTER: RANK-1-MODIFICATION FOR LINEAR EFFORT (Simeon - Numerik 1)
    Mat c = slvLU(C,rs);
    Mat ctmp(N,f.cols());
    ctmp.row(N-1) = c.row(0); // CN = c1
    ctmp(0,0,N-2,f.cols()-1)= c;

    // vector ordering of the further coefficients
    Mat d(N-1,f.cols(),INIT,0.0);
    Mat b(N-1,f.cols(),INIT,0.0);
    Mat a(N-1,f.cols(),INIT,0.0);

    for(int i=0; i<N-1; i++) {
      hi = x(i+1)-x(i);  
      a.row(i) = f.row(i);
      d.row(i) = (ctmp.row(i+1) - ctmp.row(i) ) / 3 / hi;
      b.row(i) = (f.row(i+1)-f.row(i)) / hi - (ctmp.row(i+1) + 2*ctmp.row(i) ) / 3 * hi;
    }

    breaks.resize(N);
    breaks = x.copy();
    coefs.push_back(d);
    coefs.push_back(c);
    coefs.push_back(b);
    coefs.push_back(a);
  }

  void PPolynom::calculateSplineNatural(const Vec &x, const Mat &f) {
    // first row
    int i=0;
    int N = x.size();
    SqrMat C(N-2,N-2,INIT,0.0);
    Mat rs(N-2,f.cols(),INIT,0.0);
    double hi = x(i+1)-x(i);
    double hii = x(i+2)-x(i+1);
    C(i,i) = 2*hi+2*hii;
    C(i,i+1) = hii;
    rs.row(i) = 3*(f.row(i+2)-f.row(i+1))/hii - 3*(f.row(i+1)-f.row(i))/hi;    

    // last row
    i = (N-3);
    hi = x(i+1)-x(i);
    hii = x(i+2)-x(i+1);
    C(i,i-1) = hi;
    C(i,i) = 2*hii + 2*hi;
    rs.row(i) = 3*(f.row(i+2)-f.row(i+1))/hii - 3*(f.row(i+1)-f.row(i))/hi;

    for(i=1;i<N-3;i++) { 
      hi = x(i+1)-x(i);
      hii = x(i+2)-x(i+1);
      C(i,i-1) = hi;
      C(i,i) = 2*(hi+hii);
      C(i,i+1) = hii;
      rs.row(i) = 3*(f.row(i+2)-f.row(i+1))/hii - 3*(f.row(i+1)-f.row(i))/hi;
    }

    // solve C*c = rs with C tridiagonal
    Mat C_rs(N-2,N-1+f.cols(),INIT,0.0);  
    C_rs(0,0,N-3,N-3) = C; // C_rs=[C rs] for Gauss in matrix
    C_rs(0,N-2,N-3,N-2+f.cols()-1) = rs;
    for(i=1; i<N-2; i++) C_rs.row(i) = C_rs.row(i) - C_rs.row(i-1)*C_rs(i,i-1)/C_rs(i-1,i-1); // C_rs -> upper triangular matrix C1 -> C1 rs1
    Mat rs1 = C_rs(0,N-2,N-3,N-2+f.cols()-1);
    Mat C1 = C_rs(0,0,N-3,N-3);
    Mat c(N-2,f.cols(),INIT,0.0);
    for(i=N-3;i>=0 ;i--) { // backward substitution
      RowVec sum_ciCi(f.cols(),INIT,0.); 
      for(int ii=i+1; ii<=N-3; ii++) sum_ciCi = sum_ciCi + C1(i,ii)*c.row(ii);
      c.row(i)= (rs1.row(i) - sum_ciCi)/C1(i,i);
    }
    Mat ctmp(N,f.cols(),INIT,0.0);
    ctmp(1,0,N-2,f.cols()-1) = c; // c1=cN=0 natural splines c=[ 0; c; 0]

    // vector ordering of the further coefficients
    Mat d(N-1,f.cols(),INIT,0.0);
    Mat b(N-1,f.cols(),INIT,0.0);
    Mat a(N-1,f.cols(),INIT,0.0);

    for(i=0; i<N-1; i++) {
      hi = x(i+1)-x(i);  
      a.row(i) = f.row(i);
      d.row(i) = (ctmp.row(i+1) - ctmp.row(i) ) / 3 / hi;
      b.row(i) = (f.row(i+1)-f.row(i)) / hi - (ctmp.row(i+1) + 2*ctmp.row(i) ) / 3 * hi;
    }

    breaks.resize(N);
    breaks = x.copy();
    coefs.push_back(d);
    coefs.push_back(ctmp(0,0,N-2,f.cols()-1));
    coefs.push_back(b);
    coefs.push_back(a);
  }

  void PPolynom::calculatePLinear(const Vec &x, const Mat &f) {
    int N = x.size(); // number of supporting points

    breaks.resize(N);
    breaks = x.copy();

    Mat m(N-1,f.cols(),INIT,0.0);
    Mat a(N-1,f.cols(),INIT,0.0);
    for(int i=1;i<N;i++) {
      m.row(i-1) = (f.row(i)-f.row(i-1))/(x(i)-x(i-1)); // slope
      a.row(i-1) = f.row(i-1);
    }
    coefs.push_back(m);
    coefs.push_back(a);
  }
          
  Vec PPolynom::ZerothDerivative::operator()(const double& x, const void *) {
    if(x>(parent->breaks)(parent->nPoly)) 
      throw MBSimError("ERROR (PPolynom::operator()): x out of range! x= "+numtostr(x)+", upper bound= "+numtostr((parent->breaks)(parent->nPoly)));
    if(x<(parent->breaks)(0)) 
      throw MBSimError("ERROR (PPolynom::operator()): x out of range! x= "+numtostr(x)+", lower bound= "+numtostr((parent->breaks)(0)));

    if ((fabs(x-xSave)<macheps()) && ySave.size())
      return ySave;
    else {
      if(x<(parent->breaks)(parent->index)) // saved index still OK? otherwise search downwards
        while((parent->index) > 0 && (parent->breaks)(parent->index) > x)
          (parent->index)--;
      else if(x>(parent->breaks)((parent->index)+1)) { // saved index still OK? otherwise search upwards
        while((parent->index) < (parent->nPoly) && (parent->breaks)(parent->index) <= x)
          (parent->index)++;
        (parent->index)--;  
      }

      const double dx = x - (parent->breaks)(parent->index); // local coordinate
      Vec yi = trans(((parent->coefs)[0]).row(parent->index).copy());
      for(int i=1;i<=(parent->order);i++) // Horner scheme
        yi = yi*dx+trans(((parent->coefs)[i]).row(parent->index));
      xSave=x;
      ySave=yi;
      return yi.copy();
    }
  }

  Vec PPolynom::FirstDerivative::operator()(const double& x, const void *) {
    if(x>(parent->breaks)(parent->nPoly)) throw MBSimError("ERROR (PPolynom::diff1): x out of range! x= "+numtostr(x)+", upper bound= "+numtostr((parent->breaks)(parent->nPoly)));
    if(x<(parent->breaks)(0)) throw MBSimError("ERROR (PPolynom::diff1): x out of range!   x= "+numtostr(x)+" lower bound= "+numtostr((parent->breaks)(0)));

    if ((fabs(x-xSave)<macheps()) && ySave.size())
      return ySave;
    else {
      if(x<(parent->breaks)(parent->index)) // saved index still OK? otherwise search downwards
        while((parent->index) > 0 && (parent->breaks)(parent->index) > x)
          (parent->index)--;
      else if(x>(parent->breaks)((parent->index)+1)) { // saved index still OK? otherwise search upwards
        while((parent->index) < (parent->nPoly) && (parent->breaks)(parent->index) <= x)
          (parent->index)++;
        (parent->index)--;  
      }

      double dx = x - (parent->breaks)(parent->index);
      Vec yi = trans(((parent->coefs)[0]).row(parent->index).copy())*(parent->order);
      for(int i=1;i<parent->order;i++)
        yi = yi*dx+trans(((parent->coefs)[i]).row(parent->index))*((parent->order)-i);
      xSave=x;
      ySave=yi;
      return yi.copy();
    }
  }

  Vec PPolynom::SecondDerivative::operator()(const double& x, const void *) {
    if(x>(parent->breaks)(parent->nPoly)) throw MBSimError("ERROR (PPolynom::diff2): x out of range!   x= "+numtostr(x)+" upper bound= "+numtostr((parent->breaks)(parent->nPoly)));
    if(x<(parent->breaks)(0)) throw MBSimError("ERROR (PPolynom::diff2): x out of range!   x= "+numtostr(x)+" lower bound= "+numtostr((parent->breaks)(0)));

    if ((fabs(x-xSave)<macheps()) && ySave.size())
      return ySave;
    else {
      if(x<(parent->breaks)(parent->index)) // saved index still OK? otherwise search downwards
        while((parent->index) > 0 && (parent->breaks)(parent->index) > x)
          (parent->index)--;
      else if(x>(parent->breaks)((parent->index)+1)) { // saved index still OK? otherwise search upwards
        while((parent->index) < (parent->nPoly) && (parent->breaks)(parent->index) <= x)
          (parent->index)++;
        (parent->index)--;  
      }

      double dx = x - (parent->breaks)(parent->index);
      Vec yi = trans(((parent->coefs)[0]).row(parent->index).copy())*(parent->order)*((parent->order)-1);
      for(int i=1;i<=((parent->order)-2);i++)
        yi = yi*dx+trans(((parent->coefs)[i]).row(parent->index))*((parent->order)-i)*((parent->order)-i-1);
      xSave=x;
      ySave=yi;
      return yi.copy();
    }
  }

  void PPolynom::initializeUsingXML(TiXmlElement * element) {
    DifferentiableFunction1<Vec>::initializeUsingXML(element);
    TiXmlElement * e;
    Vec x;
    Mat y;
    e=element->FirstChildElement(MBSIMNS"x");
    if (e) {
      x=Element::getVec(e);
      e=element->FirstChildElement(MBSIMNS"y");
      y=Element::getMat(e, x.size(), 0);
    }
    else {
      e=element->FirstChildElement(MBSIMNS"xy");
      Mat xy=Element::getMat(e);
      assert(xy.cols()>1);
      x=xy.col(0);
      y=xy(0, 1, xy.rows()-1, xy.cols()-1);
    }
    string method;
    if (element->FirstChildElement(MBSIMNS"cSplinePeriodic"))
      method="csplinePer";
    else if (element->FirstChildElement(MBSIMNS"cSplineNatural"))
      method="csplineNat";
    else if (element->FirstChildElement(MBSIMNS"piecewiseLinear"))
      method="plinear";
    setXF(x, y, method);
  }

}

