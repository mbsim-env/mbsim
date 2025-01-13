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

  template<typename Ret>
  void PiecewisePolynomFunction<Ret(double)>::calculateSplinePeriodic() {
    double hi, hii;
    int N = x.size();
    if(nrm2(y.row(0)-y.row(y.rows()-1))>epsroot) this->throwError("(PiecewisePolynomFunction::calculateSplinePeriodic): f(0)= "+fmatvec::toString(y.row(0))+"!="+fmatvec::toString(y.row(y.rows()-1))+" =f(end)");
    fmatvec::SqrMat C(N-1,fmatvec::INIT,0.0);
    fmatvec::Mat rs(N-1,y.cols(),fmatvec::INIT,0.0);

    // Matrix C and vector rs C*c=rs
    for(int i=0; i<N-3;i++) {
      hi = x(i+1) - x(i);
      hii = x(i+2)-x(i+1);
      C(i,i) = hi;
      C(i,i+1) = 2*(hi+hii);
      C(i,i+2) = hii;
      rs.set(i, 3.*((y.row(i+2)-y.row(i+1))/hii - (y.row(i+1)-y.row(i))/hi));
    }

    // last but one row
    hi = x(N-2)-x(N-3);
    hii = x(N-1)-x(N-2);
    C(N-3,N-3) = hi;
    C(N-3,N-2)= 2*(hi+hii);
    C(N-3,0)= hii;
    rs.set(N-3, 3.*((y.row(N-1)-y.row(N-2))/hii - (y.row(N-2)-y.row(N-3))/hi));

    // last row
    double h1 = x(1)-x(0);
    double hN_1 = x(N-1)-x(N-2);
    C(N-2,0) = 2*(h1+hN_1);
    C(N-2,1) = h1;
    C(N-2,N-2)= hN_1;
    rs.set(N-2, 3.*((y.row(1)-y.row(0))/h1 - (y.row(0)-y.row(N-2))/hN_1));

    // solve C*c = rs -> TODO BETTER: RANK-1-MODIFICATION FOR LINEAR EFFORT (Simeon - Numerik 1)
    fmatvec::Mat c = slvLU(C,rs);
    fmatvec::Mat ctmp(N,y.cols());
    ctmp.set(N-1, c.row(0)); // CN = c1
    ctmp.set(fmatvec::RangeV(0,N-2),fmatvec::RangeV(0,y.cols()-1), c);

    // vector ordering of the further coefficients
    fmatvec::Mat d(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat b(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat a(N-1,y.cols(),fmatvec::INIT,0.0);

    for(int i=0; i<N-1; i++) {
      hi = x(i+1)-x(i);  
      a.set(i, y.row(i));
      d.set(i, (ctmp.row(i+1) - ctmp.row(i) ) / 3. / hi);
      b.set(i, (y.row(i+1)-y.row(i)) / hi - (ctmp.row(i+1) + 2.*ctmp.row(i) ) / 3. * hi);
    }

    breaks.resize(N);
    breaks = x;
    coefs.clear();
    coefs.push_back(d);
    coefs.push_back(c);
    coefs.push_back(b);
    coefs.push_back(a);
  }

  template<typename Ret>
  void PiecewisePolynomFunction<Ret(double)>::calculateSplineNatural() {
    // first row
    int i=0;
    int N = x.size();
    if(N<4) this->throwError("(PiecewisePolynomFunction::calculateSplineNatural): number of datapoints does not match, must be greater 3");
    fmatvec::SqrMat C(N-2,fmatvec::INIT,0.0);
    fmatvec::Mat rs(N-2,y.cols(),fmatvec::INIT,0.0);
    double hi = x(i+1)-x(i);
    double hii = x(i+2)-x(i+1);
    C(i,i) = 2*hi+2*hii;
    C(i,i+1) = hii;
    rs.set(i, 3.*(y.row(i+2)-y.row(i+1))/hii - 3.*(y.row(i+1)-y.row(i))/hi);

    // last row
    i = (N-3);
    hi = x(i+1)-x(i);
    hii = x(i+2)-x(i+1);
    C(i,i-1) = hi;
    C(i,i) = 2*hii + 2*hi;
    rs.set(i, 3.*(y.row(i+2)-y.row(i+1))/hii - 3.*(y.row(i+1)-y.row(i))/hi);

    for(i=1;i<N-3;i++) { 
      hi = x(i+1)-x(i);
      hii = x(i+2)-x(i+1);
      C(i,i-1) = hi;
      C(i,i) = 2*(hi+hii);
      C(i,i+1) = hii;
      rs.set(i, 3.*(y.row(i+2)-y.row(i+1))/hii - 3.*(y.row(i+1)-y.row(i))/hi);
    }

    // solve C*c = rs with C tridiagonal
    fmatvec::Mat C_rs(N-2,N-1+y.cols(),fmatvec::INIT,0.0);
    C_rs.set(fmatvec::RangeV(0,N-3),fmatvec::RangeV(0,N-3), C); // C_rs=[C rs] for Gauss in matrix
    C_rs.set(fmatvec::RangeV(0,N-3),fmatvec::RangeV(N-2,N-2+y.cols()-1), rs);
    for(i=1; i<N-2; i++) C_rs.set(i, C_rs.row(i) - C_rs.row(i-1)*C_rs(i,i-1)/C_rs(i-1,i-1)); // C_rs -> upper triangular matrix C1 -> C1 rs1
    fmatvec::Mat rs1 = C_rs(fmatvec::RangeV(0,N-3),fmatvec::RangeV(N-2,N-2+y.cols()-1));
    fmatvec::Mat C1 = C_rs(fmatvec::RangeV(0,N-3),fmatvec::RangeV(0,N-3));
    fmatvec::Mat c(N-2,y.cols(),fmatvec::INIT,0.0);
    for(i=N-3;i>=0 ;i--) { // backward substitution
      fmatvec::RowVecV sum_ciCi(y.cols(),fmatvec::NONINIT);
      sum_ciCi.init(0.);
      for(int ii=i+1; ii<=N-3; ii++) sum_ciCi = sum_ciCi + C1(i,ii)*c.row(ii);
      c.set(i, (rs1.row(i) - sum_ciCi)/C1(i,i));
    }
    fmatvec::Mat ctmp(N,y.cols(),fmatvec::INIT,0.0);
    ctmp.set(fmatvec::RangeV(1,N-2),fmatvec::RangeV(0,y.cols()-1), c); // c1=cN=0 natural splines c=[ 0; c; 0]

    // vector ordering of the further coefficients
    fmatvec::Mat d(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat b(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat a(N-1,y.cols(),fmatvec::INIT,0.0);

    for(i=0; i<N-1; i++) {
      hi = x(i+1)-x(i);  
      a.set(i, y.row(i));
      d.set(i, (ctmp.row(i+1) - ctmp.row(i) ) / 3. / hi);
      b.set(i, (y.row(i+1)-y.row(i)) / hi - (ctmp.row(i+1) + 2.*ctmp.row(i) ) / 3. * hi);
    }

    breaks.resize(N);
    breaks = x;
    coefs.clear();
    coefs.push_back(d);
    coefs.push_back(ctmp(fmatvec::RangeV(0,N-2),fmatvec::RangeV(0,y.cols()-1)));
    coefs.push_back(b);
    coefs.push_back(a);
  }

  template<typename Ret>
  void PiecewisePolynomFunction<Ret(double)>::calculatePLinear() {
    int N = x.size(); // number of supporting points

    breaks.resize(N);
    breaks = x;

    fmatvec::Mat m(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat a(N-1,y.cols(),fmatvec::INIT,0.0);
    for(int i=1;i<N;i++) {
      m.set(i-1, (y.row(i)-y.row(i-1))/(x(i)-x(i-1))); // slope
      a.set(i-1, y.row(i-1));
    }
    coefs.clear();
    coefs.push_back(m);
    coefs.push_back(a);
  }
          
  template<typename Ret>
  Ret PiecewisePolynomFunction<Ret(double)>::ZerothDerivative::operator()(const double& x) {
    if(parent->extrapolationMethod==error) {
      if(x-1e-13>(parent->breaks)(parent->nPoly)) 
        throw std::runtime_error("(PiecewisePolynomFunction::operator()): x out of range! x= "+fmatvec::toString(x)+", upper bound= "+fmatvec::toString((parent->breaks)(parent->nPoly)));
      if(x+1e-13<(parent->breaks)(0)) 
        throw std::runtime_error("(PiecewisePolynomFunction::operator()): x out of range! x= "+fmatvec::toString(x)+", lower bound= "+fmatvec::toString((parent->breaks)(0)));
    }
    if(parent->extrapolationMethod==linear && (x<parent->breaks(0) || x>parent->breaks(parent->nPoly))) {
      int idx = x<parent->breaks(0) ? 0 : parent->nPoly;
      auto xv = FromDouble<double>::cast(parent->breaks(idx));
      auto value = parent->f(xv);
      auto der = parent->fd(xv);
      return value + der * (x - parent->breaks(idx));
    }

    if ((fabs(x-xSave)<macheps) && !firstCall)
      return FromVecV<Ret>::cast(ySave);
    else {
      firstCall = false;
      if(x<(parent->breaks)(parent->index)) // saved index still OK? otherwise search downwards
        while((parent->index) > 0 && (parent->breaks)(parent->index) > x)
          (parent->index)--;
      else if(x>(parent->breaks)((parent->index)+1)) { // saved index still OK? otherwise search upwards
        while((parent->index) < (parent->nPoly) && (parent->breaks)(parent->index) <= x)
          (parent->index)++;
        (parent->index)--;  
      }

      const double dx = x - (parent->breaks)(parent->index); // local coordinate
      fmatvec::VecV yi = trans(((parent->coefs)[0]).row(parent->index));
      for(int i=1;i<=(parent->order);i++) // Horner scheme
        yi = yi*dx+trans(((parent->coefs)[i]).row(parent->index));
      xSave=x;
      ySave <<= yi;
      return FromVecV<Ret>::cast(yi);
    }
  }

  template<typename Ret>
  Ret PiecewisePolynomFunction<Ret(double)>::FirstDerivative::operator()(const double& x) {
    if(parent->extrapolationMethod==error) {
      if(x-1e-13>(parent->breaks)(parent->nPoly)) throw std::runtime_error("(PiecewisePolynomFunction::diff1): x out of range! x= "+fmatvec::toString(x)+", upper bound= "+fmatvec::toString((parent->breaks)(parent->nPoly)));
      if(x+1e-13<(parent->breaks)(0)) throw std::runtime_error("(PiecewisePolynomFunction::diff1): x out of range!   x= "+fmatvec::toString(x)+" lower bound= "+fmatvec::toString((parent->breaks)(0)));
    }
    if(parent->extrapolationMethod==linear && (x<parent->breaks(0) || x>parent->breaks(parent->nPoly))) {
      int idx = x<parent->breaks(0) ? 0 : parent->nPoly;
      auto xv = FromDouble<double>::cast(parent->breaks(idx));
      auto der = parent->fd(xv);
      return der;
    }

    if ((fabs(x-xSave)<macheps) && !firstCall)
      return FromVecV<Ret>::cast(ySave);
    else {
      firstCall = false;
      if(x<(parent->breaks)(parent->index)) // saved index still OK? otherwise search downwards
        while((parent->index) > 0 && (parent->breaks)(parent->index) > x)
          (parent->index)--;
      else if(x>(parent->breaks)((parent->index)+1)) { // saved index still OK? otherwise search upwards
        while((parent->index) < (parent->nPoly) && (parent->breaks)(parent->index) <= x)
          (parent->index)++;
        (parent->index)--;  
      }

      double dx = x - (parent->breaks)(parent->index);
      fmatvec::VecV yi = trans(((parent->coefs)[0]).row(parent->index))*double(parent->order);
      for(int i=1;i<parent->order;i++)
        yi = yi*dx+trans(((parent->coefs)[i]).row(parent->index))*double((parent->order)-i);
      xSave=x;
      ySave <<= yi;
      return FromVecV<Ret>::cast(yi);
    }
  }

  template<typename Ret>
  Ret PiecewisePolynomFunction<Ret(double)>::SecondDerivative::operator()(const double& x) {
    if(parent->extrapolationMethod==error) {
      if(x-1e-13>(parent->breaks)(parent->nPoly)) throw std::runtime_error("(PiecewisePolynomFunction::diff2): x out of range!   x= "+fmatvec::toString(x)+" upper bound= "+fmatvec::toString((parent->breaks)(parent->nPoly)));
      if(x+1e-13<(parent->breaks)(0)) throw std::runtime_error("(PiecewisePolynomFunction::diff2): x out of range!   x= "+fmatvec::toString(x)+" lower bound= "+fmatvec::toString((parent->breaks)(0)));
    }
    if(parent->extrapolationMethod==linear && (x<parent->breaks(0) || x>parent->breaks(parent->nPoly)))
      return FromVecV<Ret>::cast(fmatvec::VecV(parent->getRetSize().first, fmatvec::INIT, 0.0));

    if ((fabs(x-xSave)<macheps) && !firstCall)
      return FromVecV<Ret>::cast(ySave);
    else {
      firstCall = false;
      if(x<(parent->breaks)(parent->index)) // saved index still OK? otherwise search downwards
        while((parent->index) > 0 && (parent->breaks)(parent->index) > x)
          (parent->index)--;
      else if(x>(parent->breaks)((parent->index)+1)) { // saved index still OK? otherwise search upwards
        while((parent->index) < (parent->nPoly) && (parent->breaks)(parent->index) <= x)
          (parent->index)++;
        (parent->index)--;  
      }

      double dx = x - (parent->breaks)(parent->index);
      fmatvec::VecV yi = trans(((parent->coefs)[0]).row(parent->index))*double(parent->order)*double((parent->order)-1);
      for(int i=1;i<=((parent->order)-2);i++)
        yi = yi*dx+trans(((parent->coefs)[i]).row(parent->index))*double((parent->order)-i)*double((parent->order)-i-1);
      xSave=x;
      ySave <<= yi;
      return FromVecV<Ret>::cast(yi);
    }
  }

  template<typename Ret>
  void PiecewisePolynomFunction<Ret(double)>::setCoefficients(const std::vector<fmatvec::MatV> &allCoefs) {
    interpolationMethod=useBreaksAndCoefs;

    // read all coefficient and convert to coefs
    coefs.resize(allCoefs[0].cols());
    for(auto &x : coefs)
      x.resize(allCoefs[0].rows(),allCoefs.size());
    for(size_t c=0; c<allCoefs.size(); ++c) {
      if(allCoefs[c].cols()!=static_cast<int>(coefs.size()))
        this->throwError("The number of columns in the coefficients elements differ.");
      if(allCoefs[c].rows()!=static_cast<int>(coefs[0].rows()))
        this->throwError("The number of rows in the coefficients elements differ.");
      for(int deg=0; deg<allCoefs[c].cols(); deg++)
        coefs[deg].set(c, allCoefs[c].col(deg));
      c++;
    }
  }

  template<typename Ret>
  void PiecewisePolynomFunction<Ret(double)>::addCoefficients(const fmatvec::MatV &coef) {
    interpolationMethod=useBreaksAndCoefs;

    // read all coefficient and convert to coefs
    if(coefs.size()==0)
      coefs.resize(coef.cols());
    if(static_cast<int>(coefs.size())!=coef.cols())
      this->throwError("The added coefficients have a different spline order.");
    for(auto &x : coefs) {
      auto xold(std::move(x));
      x.resize(coef.rows(),xold.cols()+1);
      x.set(fmatvec::RangeV(0,xold.rows()-1), fmatvec::RangeV(0,xold.cols()-1), std::move(xold));
    }
    if(coef.cols()!=static_cast<int>(coefs.size()))
      this->throwError("The number of columns in the coefficients elements differ.");
    if(coef.rows()!=static_cast<int>(coefs[0].rows()))
      this->throwError("The number of rows in the coefficients elements differ.");
    for(int deg=0; deg<coef.cols(); deg++)
      coefs[deg].set(coefs[deg].cols()-1, coef.col(deg));
  }

  template<typename Ret>
  void PiecewisePolynomFunction<Ret(double)>::initializeUsingXML(xercesc::DOMElement * element) {
    xercesc::DOMElement *e;
    fmatvec::VecV x;
    fmatvec::MatV y;
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"x");
    if (e) {
      setx(MBXMLUtils::E(e)->getText<fmatvec::Vec>());
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"y");
      sety(MBXMLUtils::E(e)->getText<fmatvec::Mat>(x.size(), 0));
    }
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"xy");
    if (e) {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"xy");
      setxy(MBXMLUtils::E(e)->getText<fmatvec::Mat>());
    }
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"interpolationMethod");
    if(e) { 
      std::string str=MBXMLUtils::X()%MBXMLUtils::E(e)->getFirstTextChild()->getData();
      str=str.substr(1,str.length()-2);
      if(str=="cSplinePeriodic") interpolationMethod=cSplinePeriodic;
      else if(str=="cSplineNatural") interpolationMethod=cSplineNatural;
      else if(str=="piecewiseLinear") interpolationMethod=piecewiseLinear;
      else interpolationMethod=useBreaksAndCoefs;
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"breaks");
    if(e) { 
      // set breaks
      setBreaks(MBXMLUtils::E(e)->getText<fmatvec::Vec>());

      // read all coefficients elements
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"coefficients");
      std::vector<fmatvec::MatV> allCoefs;
      while(e) {
        allCoefs.emplace_back(MBXMLUtils::E(e)->getText<fmatvec::Mat>());
        e=MBXMLUtils::E(e)->getNextElementSiblingNamed(MBSIM%"coefficients");
      }
      setCoefficients(allCoefs);
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"extrapolationMethod");
    if(e) { 
      std::string str=MBXMLUtils::X()%MBXMLUtils::E(e)->getFirstTextChild()->getData();
      str=str.substr(1,str.length()-2);
      if(str=="error")      extrapolationMethod=error;
      else if(str=="continuePolynom")  extrapolationMethod=continuePolynom;
      else if(str=="linear") extrapolationMethod=linear;
      else this->throwError("Unknown extrapolationMethod.");
    }
  }
