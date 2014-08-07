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
#include "mbsim/utils/eps.h"
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

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Eigenanalyser, MBSIMANALYSER%"Eigenanalyser")

  Eigenanalyser::Residuum::Residuum(DynamicSystemSolver *sys_, double t_) : sys(sys_), t(t_) {}

  Vec Eigenanalyser::Residuum::operator()(const Vec &z) {
      Vec res;
      res = sys->zdot(z,t);
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
      if(not(zEq.size())) {
        zEq.resize(system->getzSize());
        system->initz(zEq);          
      }

      if(compEq) {
        Residuum f(system,tStart);
        MultiDimNewtonMethod newton(&f);
        newton.setLinearAlgebra(1);
        zEq = newton.solve(zEq);
        if(newton.getInfo() != 0)
          throw MBSimError("ERROR in Eigenanalysis: computation of equilibrium state failed!");
      }

      double delta = epsroot();
      SqrMat A(zEq.size());
      Vec zd, zdOld;
      zdOld = system->zdot(zEq,tStart);
      for (int i=0; i<zEq.size(); i++) {
        double ztmp = zEq(i);
        zEq(i) += delta;
        zd = system->zdot(zEq,tStart);
        A.col(i) = (zd - zdOld) / delta;
        zEq(i) = ztmp;
      }
      eigvec(A,V,w);
      computeEigenfrequencies();
      saveEigenanalyis(fileName.empty()?system->getName()+".eigenanalysis.mat":fileName);
  }

  void Eigenanalyser::computeEigenmodes() {
    if(autoUpdate) computeEigenvalues();
    else if(not(w.size()) and not(loadEigenanalyis(fileName.empty()?system->getName()+".eigenanalysis.mat":fileName)))
      throw MBSimError("ERROR in Eigenanalysis: eigenanalysis not yet performed!");
    Vector<Ref, complex<double> > c(w.size());
    Vector<Ref, complex<double> > deltaz(w.size(),NONINIT);
    Vector<Ref, complex<double> > wbuf;
    wbuf = w;

    Vec z;
    double t0 = tStart;
    for(int j=0; j<f.size(); j++) {
      c(f[j].second) = complex<double>(0,A);
      c(f[j].second+1) = complex<double>(0,-A);
      real(wbuf(f[j].second)) = 0;
      real(wbuf(f[j].second+1)) = 0;
      double T=10*1./f[j].first;
      for(double t=t0; t<t0+T+dtPlot; t+=dtPlot) {
        deltaz.init(0);
        for(int i=0; i<wbuf.size(); i++)
          deltaz += c(i)*V.col(i)*exp(wbuf(i)*t); 
        z = zEq + fromComplex(deltaz);
        system->plot(z,t);
      }
      t0 += T+dtPlot;
      c(f[j].second) = complex<double>(0,0);
      c(f[j].second+1) = complex<double>(0,0);
    }
  }

  void Eigenanalyser::computeEigenmode() {
    if(autoUpdate) computeEigenvalues();
    else if(not(w.size()) and not(loadEigenanalyis(fileName.empty()?system->getName()+".eigenanalysis.mat":fileName)))
      throw MBSimError("ERROR in Eigenanalysis: eigenanalysis not yet performed!");
    if(n<1 or n>f.size())
      throw MBSimError("ERROR in Eigenanalysis: frequency number out of range!");
    Vector<Ref, complex<double> > c(w.size());
    Vector<Ref, complex<double> > deltaz(w.size(),NONINIT);
    Vector<Ref, complex<double> > wbuf;
    wbuf = w;

    Vec z;
    c(f[n-1].second) = complex<double>(0,A);
    c(f[n-1].second+1) = complex<double>(0,-A);
    real(wbuf(f[n-1].second)) = 0;
    real(wbuf(f[n-1].second+1)) = 0;
    double T=1./f[n-1].first;
    for(double t=tStart; t<tStart+T+dtPlot; t+=dtPlot) {
      deltaz.init(0);
      for(int i=0; i<wbuf.size(); i++)
        deltaz += c(i)*V.col(i)*exp(wbuf(i)*t); 
      z = zEq + fromComplex(deltaz);
      system->plot(z,t);
    }
  }

  void Eigenanalyser::computeEigenmotion() {
    if(autoUpdate) computeEigenvalues();
    else if(not(w.size()) and not(loadEigenanalyis(fileName.empty()?system->getName()+".eigenanalysis.mat":fileName)))
      throw MBSimError("ERROR in Eigenanalysis: eigenanalysis not yet performed!");
    Vector<Ref, complex<double> > deltaz(w.size(),NONINIT);

    Vec z;
    if(deltaz0.size()==0)
      deltaz0.resize(w.size());
    Vector<Ref, complex<double> > c = slvLU(V,toComplex(deltaz0));

    for(double t=tStart; t<tEnd+dtPlot; t+=dtPlot) {
      deltaz.init(0);
      for(int i=0; i<w.size(); i++)
        deltaz += c(i)*V.col(i)*exp(w(i)*t); 
      z = zEq + fromComplex(deltaz);
      system->plot(z,t);
    }
  }

  bool Eigenanalyser::saveEigenanalyis(const string& fileName) {
    ofstream os(fileName.c_str());
    if(os.is_open()) {
      os << "# name: lambda" << endl;
      os << "# type: complex matrix" << endl;
      os << "# rows: " << w.size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i < w.size(); ++i)
        os << setw(26) << w.e(i) << endl;
      os << endl;
      os << "# name: V" << endl;
      os << "# type: complex matrix" << endl;
      os << "# rows: " << V.rows() << endl;
      os << "# columns: " << V.cols() << endl;
      for(int i=0; i < V.rows(); ++i) {
        for(int j=0; j < V.cols(); ++j) 
          os << setw(26) << V.e(i,j);
        os << endl;
      }
      os << endl;
      os << "# name: z" << endl;
      os << "# type: matrix" << endl;
      os << "# rows: " << freq.size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i < zEq.size(); ++i)
        os << setw(26) << zEq.e(i) << endl;
      os << endl;
      os << "# name: f" << endl;
      os << "# type: matrix" << endl;
      os << "# rows: " << freq.size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i < freq.size(); ++i)
        os << setw(26) << freq.e(i) << endl;
      os.close();
      return true;
    }
    return false;
  }

  bool Eigenanalyser::loadEigenanalyis(const string& fileName) {
    ifstream is(fileName.c_str());
    if(is.is_open()) {
      char str[100];
      string n;
      is.getline(str,100);
      is.getline(str,100);
      is >> n >> n >> n;
      w.resize(atoi(n.c_str()));
      V.resize(w.size());
      zEq.resize(w.size());
      is >> n;
      is.getline(str,100);
      for(int i=0; i<w.size(); i++)
        is >> w.e(i);
      is.getline(str,100);
      is.getline(str,100);
      is.getline(str,100);
      is.getline(str,100);
      is.getline(str,100);
      is.getline(str,100);
      for(int i=0; i<V.size(); i++)
        for(int j=0; j<V.size(); j++)
          is >> V.e(i,j);
      is.getline(str,100);
      is.getline(str,100);
      is.getline(str,100);
      is.getline(str,100);
      is.getline(str,100);
      is.getline(str,100);
      for(int i=0; i<zEq.size(); i++)
        is >> zEq.e(i);
      is.close();
      computeEigenfrequencies();
      return true;
    }
    return false;
  }

  void Eigenanalyser::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"startTime");
    setStartTime(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"endTime");
    setEndTime(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"plotStepSize");
    setPlotStepSize(Element::getDouble(e));
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
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"autoUpdate");
    if(e) setAutoUpdate(Element::getBool(e));
  }

}

