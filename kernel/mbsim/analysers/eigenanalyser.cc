/* Copyright (C) 2004-2014 MBSim Development Team
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
#include "eigenanalyser.h"
#include "mbsim/dynamic_system_solver.h"
#include "fmatvec/linear_algebra_complex.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "mbsim/utils/eps.h"
#include <iostream>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

// TODO: remove the following two functions and provide an uniform concept to
// cast between complex and double
Vector<Ref, complex<double> > toComplex(const Vector<Ref, double> &x) {
  Vector<Ref, complex<double> > y(x.size(),NONINIT);
  for(int i=0; i<x.size(); i++)
    y(i) = complex<double>(x(i),0);
  return y;
}
const Vector<Ref, double> fromComplex(const Vector<Ref, complex<double> > &x) {
  Vector<Ref, double> y(x.size(),NONINIT);
  for(int i=0; i<x.size(); i++)
    y(i) = x(i).real();
  return y;
}

namespace MBSimAnalyser {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMANALYSER, Eigenanalyser)

  Eigenanalyser::Residuum::Residuum(DynamicSystemSolver *sys_, double t_) : sys(sys_), t(t_) {}

  Vec Eigenanalyser::Residuum::operator()(const Vec &z) {
    Vec res;
    system->setTime(t);
    system->setState(z);
    system->resetUpToDate();
    res = system->evalzd();
    return res;
  } 

  void Eigenanalyser::analyse(DynamicSystemSolver& system_) {
    system = &system_;

    if(task == eigenfrequencies) computeEigenvalues();
    else if(task == eigenmode) computeEigenmode();
    else if(task == eigenmodes) computeEigenmodes();
    else if(task == eigenmotion) computeEigenmotion();
  }

  void Eigenanalyser::computeEigenfrequencies() {
    f.clear();
    for (int i=0; i<w.size(); i++) {
      if((abs(imag(w(i))) > macheps()) and (i < w.size()-1) and (w(i+1)==conj(w(i)))) {
        f.push_back(pair<double,int>(imag(w(i))/2/M_PI,i));
        i++;
      }
    }
    std::sort(f.begin(), f.end());
    freq.resize(f.size(),NONINIT);
    for(int i=0; i<freq.size(); i++)
      freq.e(i) = f[i].first;
  }

  void Eigenanalyser::computeEigenvalues() {
    if(not(zEq.size()))
      zEq = system->evalz0();

    if(compEq) {
      Residuum f(system,tStart);
      MultiDimNewtonMethod newton(&f);
      newton.setLinearAlgebra(1);
      zEq = newton.solve(zEq);
      if(newton.getInfo() != 0)
        throw MBSimError("In Eigenanalysis: computation of equilibrium state failed!");
    }

    double delta = epsroot();
    SqrMat A(system->getzSize());
    Vec zd, zdOld;
    system->setTime(tStart);
    system->setState(zEq);
    system->resetUpToDate();
    zdOld = system->evalzd();
    for (int i=0; i<system->getzSize(); i++) {
      double ztmp = system->getState()(i);
      system->getState()(i) += delta;
      system->resetUpToDate();
      zd = system->evalzd();
      A.col(i) = (zd - zdOld) / delta;
      system->getState()(i) = ztmp;
    }
    eigvec(A,V,w);
    computeEigenfrequencies();
    saveEigenanalyis(fileName.empty()?system->getName()+".eigenanalysis.mat":fileName);
  }

  void Eigenanalyser::computeEigenmodes() {
    computeEigenvalues();
    Vector<Ref, complex<double> > c(w.size());
    Vector<Ref, complex<double> > deltaz(w.size(),NONINIT);
    Vector<Ref, complex<double> > wbuf;
    wbuf = w;

    double t0 = tStart;
    for(int j=0; j<static_cast<int>(f.size()); j++) {
      c(f[j].second) = complex<double>(0,A);
      c(f[j].second+1) = complex<double>(0,-A);
      wbuf(f[j].second).real(0);
      wbuf(f[j].second+1).real(0);
      double T=10*1./f[j].first;
      for(double t=t0; t<t0+T+dtPlot; t+=dtPlot) {
        deltaz.init(0);
        for(int i=0; i<wbuf.size(); i++)
          deltaz += c(i)*V.col(i)*exp(wbuf(i)*t); 
        system->setTime(t);
        system->setState(zEq + fromComplex(deltaz));
        system->resetUpToDate();
        system->plot();
      }
      t0 += T+dtPlot;
      c(f[j].second) = complex<double>(0,0);
      c(f[j].second+1) = complex<double>(0,0);
    }
  }

  void Eigenanalyser::computeEigenmode() {
    computeEigenvalues();
    if(n<1 or n>static_cast<int>(f.size()))
      throw MBSimError("In Eigenanalysis: frequency number out of range!");
    Vector<Ref, complex<double> > c(w.size());
    Vector<Ref, complex<double> > deltaz(w.size(),NONINIT);
    Vector<Ref, complex<double> > wbuf;
    wbuf = w;

    c(f[n-1].second) = complex<double>(0,A);
    c(f[n-1].second+1) = complex<double>(0,-A);
    wbuf(f[n-1].second).real(0);
    wbuf(f[n-1].second+1).real(0);
    double T=1./f[n-1].first;
    for(double t=tStart; t<tStart+T+dtPlot; t+=dtPlot) {
      deltaz.init(0);
      for(int i=0; i<wbuf.size(); i++)
        deltaz += c(i)*V.col(i)*exp(wbuf(i)*t); 
      system->setTime(t);
      system->setState(zEq + fromComplex(deltaz));
      system->resetUpToDate();
      system->plot();
    }
  }

  void Eigenanalyser::computeEigenmotion() {
    computeEigenvalues();
    Vector<Ref, complex<double> > deltaz(w.size(),NONINIT);

    if(deltaz0.size()==0)
      deltaz0.resize(w.size());
    Vector<Ref, complex<double> > c = slvLU(V,toComplex(deltaz0));

    for(double t=tStart; t<tEnd+dtPlot; t+=dtPlot) {
      deltaz.init(0);
      for(int i=0; i<w.size(); i++)
        deltaz += c(i)*V.col(i)*exp(w(i)*t); 
      system->setTime(t);
      system->setState(zEq + fromComplex(deltaz));
      system->resetUpToDate();
      system->plot();
    }
  }

  bool Eigenanalyser::saveEigenanalyis(const string& fileName) {
    ofstream os(fileName.c_str());
    if(os.is_open()) {
      os << "# name: " << "lambda" << endl;
      os << "# type: " << "complex matrix" << endl;
      os << "# rows: " << w.size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i<w.size(); i++)
        os << setw(28) << w.e(i) << endl;
      os << endl;
      os << "# name: " << "V" << endl;
      os << "# type: " << "complex matrix" << endl;
      os << "# rows: " << V.rows() << endl;
      os << "# columns: " << V.cols() << endl;
      for(int i=0; i<V.rows(); i++) {
        for(int j=0; j<V.cols(); j++)
          os << setw(28) << V.e(i,j) << " ";
        os << endl;
      }
      os << endl;
      os << "# name: " << "z" << endl;
      os << "# type: " << "matrix" << endl;
      os << "# rows: " << zEq.size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i<zEq.size(); i++)
        os << setw(28) << zEq.e(i) << endl;
      os << endl;
      os << "# name: " << "f" << endl;
      os << "# type: " << "matrix" << endl;
      os << "# rows: " << freq.size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i<freq.size(); i++)
        os << setw(28) << freq.e(i) << endl;
      os << endl;
      os.close();
      return true;
    }
    return false;
  }

  void Eigenanalyser::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"startTime");
    if(e) setStartTime(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"endTime");
    if(e) setEndTime(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"plotStepSize");
    if(e) setPlotStepSize(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"initialState");
    if(e) setInitialState(Element::getVec(e));
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"task");
    if(e) {
      string str=X()%E(e)->getFirstTextChild()->getData();
      str=str.substr(1,str.length()-2);
      if(str=="eigenfrequencies") task=eigenfrequencies;
      else if(str=="eigenmodes") task=eigenmodes;
      else if(str=="eigenmode") task=eigenmode;
      else if(str=="eigenmotion") task=eigenmotion;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"amplitude");
    if(e) setAmplitude(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"mode");
    if(e) setMode(Element::getInt(e));
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"determineEquilibriumState");
    if(e) setDetermineEquilibriumState(Element::getBool(e));
  }

}
