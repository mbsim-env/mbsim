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

namespace MBSimAnalyser {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMANALYSER, HarmonicResponseAnalyser)

  HarmonicResponseAnalyser::Residuum::Residuum(DynamicSystemSolver *sys_, double t_) : sys(sys_), t(t_) { }

  Vec HarmonicResponseAnalyser::Residuum::operator()(const Vec &z) {
    Vec res;
    sys->setTime(t);
    sys->setState(z);
    sys->resetUpToDate();
    res = sys->evalzd();
    return res;
  } 

  void HarmonicResponseAnalyser::analyse() {
    if(task == frequencyResponse) computeFrequencyResponse();
  }

  void HarmonicResponseAnalyser::computeFrequencyResponse() {
    if(not(fE.size()))
      fE.resize(1,INIT,1);

    if(not(zEq.size()))
      zEq = system->evalz0();

    if(compEq) {
      Residuum f(system,tStart);
      MultiDimNewtonMethod newton(&f);
      newton.setLinearAlgebra(1);
      zEq = newton.solve(zEq);
      if(newton.getInfo() != 0)
        throw MBSimError("In harmonic response analysis: computation of equilibrium state failed!");
    }

    int n = system->getzSize();
    system->setState(zEq);

    double T = 1./fS(0);
    
    Vec y0, y1;
    Vec bri(2*n);
    Vec br = bri(0,n-1);
    Vec bi = bri(n,2*n-1);
    //system->setTime(tStart);
    //system->resetUpToDate();
    //y0 = system->evalzd()(n/2,n-1);
    //system->setTime(tStart+0.3*T);
    //system->resetUpToDate();
    //y1 = system->evalzd()(n/2,n-1);
    //br(n/2,n-1) = (y1*sin(Om*tStart) - y0*sin(Om*(tStart+0.3*T)))/sin(-0.3*Om*T);
    //bi(n/2,n-1) = (y1*cos(Om*tStart) - y0*cos(Om*(tStart+0.3*T)))/sin(0.3*Om*T);
    system->setTime(0);
    system->resetUpToDate();
    br(n/2,n-1) = system->evalzd()(n/2,n-1);
    system->setTime(0.25*T);
    system->resetUpToDate();
    bi(n/2,n-1) = system->evalzd()(n/2,n-1);

    SqrMat A(n);
    Vec zd, zdOld;
    system->setTime(tStart);
    system->resetUpToDate();
    zdOld = system->evalzd();
    for (int i=0; i<n; i++) {
      double ztmp = system->getState()(i);
      system->getState()(i) += epsroot;
      system->resetUpToDate();
      zd = system->evalzd();
      A.col(i) = (zd - zdOld) / epsroot;
      system->getState()(i) = ztmp;
    }
    SqrMat Q(2*n);
    Q(Range<Var,Var>(0,n-1),Range<Var,Var>(0,n-1)) = -A;
    Q(Range<Var,Var>(n,2*n-1),Range<Var,Var>(n,2*n-1)) = -A;
    Vec zhri(2*n,NONINIT);
    Vec zhr = zhri(0,n-1);
    Vec zhi = zhri(n,2*n-1);
//    int N = int((fE-fS)/df)+1;
    Zh.resize(fE.size(),n,NONINIT);

    for(int k=0; k<fE.size(); k++) {
      double Om = 2*M_PI*fE(k);
      Q(Range<Var,Var>(0,n-1),Range<Var,Var>(n,2*n-1)) = -Om*SqrMat(n,EYE);
      Q(Range<Var,Var>(n,2*n-1),Range<Var,Var>(0,n-1)) = Om*SqrMat(n,EYE);
      zhri = slvLU(Q,bri);
      for(int i=0; i<n; i++)
        Zh(k,i) = sqrt(pow(zhr(i),2) + pow(zhi(i),2));
    }
    ofstream os((system->getName()+".harmonic_response_analysis.mat").c_str());
    if(os.is_open()) {
      os << "# name: " << "z" << endl;
      os << "# type: " << "matrix" << endl;
      os << "# rows: " << system->getState().size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i<zEq.size(); i++)
        os << setw(28) << zEq.e(i) << endl;
      os << endl;
      os << "# name: " << "f" << endl;
      os << "# type: " << "matrix" << endl;
      os << "# rows: " << fE.size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i<fE.size(); i++)
          os << setw(28) << fE(i) << endl;
      os << endl;
      os << "# name: " << "A" << endl;
      os << "# type: " << "matrix" << endl;
      os << "# rows: " << Zh.rows() << endl;
      os << "# columns: " << Zh.cols()/2 << endl;
      for(int i=0; i<Zh.rows(); i++) {
        for(int j=0; j<Zh.cols()/2; j++)
          os << setw(28) << Zh.e(i,j) << " ";
        os << endl;
      }
      os << endl;
      os.close();
    }
//    double t0 = tStart;
    double Om = 2*M_PI/T;
    Q(Range<Var,Var>(0,n-1),Range<Var,Var>(n,2*n-1)) = -Om*SqrMat(n,EYE);
    Q(Range<Var,Var>(n,2*n-1),Range<Var,Var>(0,n-1)) = Om*SqrMat(n,EYE);
    zhri = slvLU(Q,bri);
    for(double t=tStart; t<tStart+T+dtPlot; t+=dtPlot) {
      system->setTime(t);
      system->setState(zEq + zhr*cos(Om*t) + zhi*sin(Om*t));
      system->resetUpToDate();
      system->plot();
    }
//    t0 += T+dtPlot;
  }

  void HarmonicResponseAnalyser::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"startTime");
    if(e) setStartTime(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"excitationFrequencies");
    if(e) setExcitationFrequencies(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"systemFrequencies");
    if(e) setSystemFrequencies(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"plotStepSize");
    if(e) setPlotStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"initialState");
    if(e) setInitialState(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"task");
    if(e) {
      string str=X()%E(e)->getFirstTextChild()->getData();
      str=str.substr(1,str.length()-2);
      if(str=="frequencyResponse") task=frequencyResponse;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMANALYSER%"determineEquilibriumState");
    if(e) setDetermineEquilibriumState(E(e)->getText<bool>());
  }

}
