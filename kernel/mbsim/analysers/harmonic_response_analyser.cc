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
#include "harmonic_response_analyser.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "mbsim/utils/eps.h"
#include <iostream>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

extern "C" {
#define DFFTI FC_FUNC(dffti,DFFTI)
  void DFFTI(int*, double*);
#define DFFTF FC_FUNC(dfftf,DFFTF)
  void DFFTF(int*, double*, double*);
}

namespace MBSimAnalyser {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMANALYSER, HarmonicResponseAnalyser)

  HarmonicResponseAnalyser::Residuum::Residuum(DynamicSystemSolver *sys_, double t_) : sys(sys_), t(t_) {}

  Vec HarmonicResponseAnalyser::Residuum::operator()(const Vec &z) {
    Vec res;
    system->setTime(t);
    system->setState(z);
    system->resetUpToDate();
    res = system->evalzd();
    return res;
  } 

  void HarmonicResponseAnalyser::analyse(DynamicSystemSolver& system_) {
    system = &system_;

    if(task == frequencyResponse) computeFrequencyResponse();
  }

  void HarmonicResponseAnalyser::computeFrequencyResponse() {
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

    int m = int(T/dt);
    int n = system->getzSize();
    SqrMat A(n);
    system->setState(zEq);
    Mat Zd(m,n,NONINIT);
    for(int i=0; i<m; i++) {
      system->setTime(tStart+dt*i);
      system->resetUpToDate();
      Vec zd = system->evalzd();
      for(int j=0; j<zd.size(); j++)
        Zd(i,j) = zd(j);
    }
    double *work = new double[2*m+15];
    DFFTI(&m,work);
    for(int i=n/2; i<n; i++)
      DFFTF(&m,Zd.col(i)(),work);

    double tEnd = tStart+dt*(m-0*1);
    double fm = 1./tEnd;
    cout << m << " " << n << " " << tStart << " " << dt << " " << tEnd << " fm " << fm << endl;

    Vec bri(2*n);
    Vec br = bri(0,n-1);
    Vec bi = bri(n,2*n-1);
    for(int j=n/2; j<n; j++) {
      br(j) = Zd(1,j)*dt*2./tEnd;
      bi(j) = Zd(2,j)*dt*2./tEnd;
    }
//    for(int i=1; i<m/2; i++) {
//      bool stop = false;
//      for(int j=n/2; j<n; j++) {
//        if(fabs(Zd(2*i-1,j))>1e-8 or fabs(Zd(2*i,j))>1e-8) {
//        cout << "real " << i << " " << j << " " << (i-1)*fm << " " << Zd(2*i-1,j)*dt*2./tEnd << endl;
//        cout << "imag " << i << " " << j << " " << (i-1)*fm << " " << Zd(2*i,j)*dt*2./tEnd << endl;
//        br(j) = Zd(2*i-1,j)*dt*2./tEnd;
//        bi(j) = Zd(2*i,j)*dt*2./tEnd;
//        stop = true;
//        }
//      }
//      if(stop) break;
//    }

    double delta = epsroot();
    Vec zd, zdOld;
    system->setTime(tStart);
    system->setState(zEq);
    system->resetUpToDate();
    zdOld = system->evalzd();
    for (int i=0; i<n; i++) {
      double ztmp = system->getState()(i);
      system->getState()(i) += delta;
      system->resetUpToDate();
      zd = system->evalzd();
      A.col(i) = (zd - zdOld) / delta;
      system->getState()(i) = ztmp;
    }
    double Om = 2*M_PI/T;
    SqrMat Q(2*n);
    Q(Range<Var,Var>(0,n-1),Range<Var,Var>(0,n-1)) = -A;
    Q(Range<Var,Var>(0,n-1),Range<Var,Var>(n,2*n-1)) = -Om*SqrMat(n,EYE);
    Q(Range<Var,Var>(n,2*n-1),Range<Var,Var>(0,n-1)) = Om*SqrMat(n,EYE);
    Q(Range<Var,Var>(n,2*n-1),Range<Var,Var>(n,2*n-1)) = -A;
    cout << Q << endl;
    cout << bri << endl;
    Vec zhri = slvLU(Q,bri);
    Vec zhr = zhri(0,n-1);
    Vec zhi = zhri(n,2*n-1);
    Vec zh(n,NONINIT);
    for(int i=0; i<n; i++)
      zh(i) = sqrt(pow(zhr(i),2) + pow(zhi(i),2));
    cout << zh << endl;

    for(double t=tStart; t<tStart+T+dtPlot; t+=dtPlot) {
      system->setTime(t);
      system->setState(zEq + zhr*cos(Om*t) + zhi*sin(Om*t));
      system->resetUpToDate();
      system->plot();
    }
  }

  bool HarmonicResponseAnalyser::saveEigenanalyis(const string& fileName) {
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
      os << "# rows: " << system->getState().size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i<system->getState().size(); i++)
        os << setw(28) << system->getState().e(i) << endl;
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

  void HarmonicResponseAnalyser::initializeUsingXML(DOMElement *element) {
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
      if(str=="frequencyResponse") task=frequencyResponse;
//      else if(str=="eigenmodes") task=eigenmodes;
//      else if(str=="eigenmode") task=eigenmode;
//      else if(str=="eigenmotion") task=eigenmotion;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"amplitude");
    if(e) setAmplitude(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"mode");
    if(e) setMode(Element::getInt(e));
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"determineEquilibriumState");
    if(e) setDetermineEquilibriumState(Element::getBool(e));
  }

}
