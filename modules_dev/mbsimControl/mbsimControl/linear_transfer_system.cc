/* Copyright (C) 2006  Mathias Bachmayer

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
 *
 * Contact:
 *   mbachmayer@gmx.de
 *
 */ 

#include "mbsimControl/linear_transfer_system.h"
#include "mbsimControl/signal_.h"
#include "mbsim/utils/utils.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  LinearTransferSystem::LinearTransferSystem(const string& name) : SignalProcessingSystem(name), R1(.002), R2(1.), c(1.) {
  }

  void LinearTransferSystem::updatedx(double t, double dt) {
    xd=(A*x+B*inputSignal->getSignal())*dt;
  }

  void LinearTransferSystem::updatexd(double t) {
    xd=A*x+B*inputSignal->getSignal();
  }

  void LinearTransferSystem::init() {
    SignalProcessingSystem::init();
    x.resize(xSize, INIT, 0); 
    output.resize(C.rows());
  }

  Vec LinearTransferSystem::calculateOutput() {
    output = (this->*calculateOutputMethod)();
    return output;
  }

  Vec LinearTransferSystem::outputMethodC() {
    return C*x;
  }

  Vec LinearTransferSystem::outputMethodD() {
    return D*inputSignal->getSignal();
  }

  Vec LinearTransferSystem::outputMethodCD() {
    return outputMethodC()+outputMethodD();
  }

  void LinearTransferSystem::showABCD() {
    cout  <<  "State space modell of Object \"" << name << "\":" << endl;
    cout  <<  "A Matrix:" << A << endl;
    cout  <<  "B Matrix:" << B << endl;
    cout  <<  "C Matrix:" << C << endl;
    cout  <<  "D Matrix:" << D << endl;
  }

  void LinearTransferSystem::setPID(double PP, double II, double DD) {
    if (DD!=0)
      calculateOutputMethod=&LinearTransferSystem::outputMethodCD;
    else
      if (II!=0) 
        calculateOutputMethod=&LinearTransferSystem::outputMethodCD;
      else
        calculateOutputMethod=&LinearTransferSystem::outputMethodD;

    if (DD==0) {
      A.resize(1, 1, INIT, 0);
      B.resize(1, 1, INIT, 1.);
      C.resize(1, 1, INIT, II);
      D.resize(1, 1, INIT, PP);
    }
    else {
      A.resize(2, 2, INIT, 0);
      A(1,1)=-1./(R1*c);
      B.resize(2, 1, INIT, 0);
      B(0,0)=1.;
      B(1,0)= 1./(R1*c);
      C.resize(1,2);
      C(0,0)=II;
      C(0,1)=-DD*R2*c/(R1*c);
      D.resize(1,1);
      D(0,0)=PP+DD*R2*c/(R1*c);
    }   
  }

  void LinearTransferSystem::setBandwidth(double Hz_fg) {
    assert(Hz_fg>0);
    double omegag=2.*M_PI*Hz_fg;
    R1=1./(omegag*c);
    R2=sqrt(R1*R1+1./(c*c));
  }

  void LinearTransferSystem::setIntegrator(double OutputGain) {
    A.resize(1, 1, INIT, 0);
    B.resize(1, 1, INIT, 1.);
    C.resize(1, 1, INIT, OutputGain);
    D.resize(1, 1, INIT, 0);
  }

  void LinearTransferSystem::setI2(double OutputGain) {
    A.resize(2, 2, INIT, 0);
    A(0,1)=1.;
    B.resize(2, 1, INIT, 0);
    B(1,0)=1.;
    C.resize(1, 2, INIT, 0);
    C(0,0)=OutputGain;
    D.resize(1, 1, INIT, 0);
  }

  void LinearTransferSystem::setPT1(double P,double T) {
    A.resize(1, 1, INIT, -1./T);
    B.resize(1, 1, INIT, 1.);
    C.resize(1, 1, INIT, P/T);
    D.resize(1, 1, INIT, 0);
  }

  void LinearTransferSystem::setGain(double P) {
    setPID(P, 0, 0);
  }

  void LinearTransferSystem::plot(double t, double dt) {
    for (int i=0; i<output.size(); i++)
      plotVector.push_back(output(i));
    for (int i=0; i<B.cols(); i++)
      plotVector.push_back(inputSignal->getSignal()(i));
    SignalProcessingSystem::plot(t,dt);
  }

  void LinearTransferSystem::initPlot() {
    for (int i=0; i<output.size(); i++)
      plotColumns.push_back("Output Sigout (" + numtostr(i) + ")");
    for (int i=0; i<B.cols(); i++)
      plotColumns.push_back("Input Signal (" + numtostr(i) + ")");
    SignalProcessingSystem::initPlot();
  }
}
