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

#include <config.h>
#include <iostream>
#include "mbsimControl/linear_transfer_system.h"
#include "mbsimControl/signal_.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, LinearTransferSystem)

  LinearTransferSystem::LinearTransferSystem(const string& name) : Signal(name), inputSignal(NULL), R1(.002), R2(1.), c(1.) {
  }

  void LinearTransferSystem::initializeUsingXML(DOMElement * element) {
    Signal::initializeUsingXML(element);
    DOMElement * e;
    DOMElement * ee;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    inputSignalString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"pidType");
    if (e) {
      ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"P");
      double p=Element::getDouble(ee);
      ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"I");
      double i=Element::getDouble(ee);
      ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"D");
      double d=Element::getDouble(ee);
      setPID(p, i, d);
    }
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"abcdType");
    if (e) {
      ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"A");
      Mat AA=Element::getMat(ee);
      ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"B");
      Mat BB=Element::getMat(ee, A.rows(), 0);
      ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"C");
      Mat CC=Element::getMat(ee, 0, A.cols());
      ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"D");
      Mat DD=Element::getMat(ee, C.rows(), B.cols());
      setABCD(AA, BB, CC, DD);
    }
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"integratorType");
    if (e) {
      ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"gain");
      double g=Element::getDouble(ee);
      setIntegrator(g);
    }
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"pt1Type");
    if (e) {
      ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"P");
      double PP=Element::getDouble(ee);
      ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"T");
      double TT=Element::getDouble(ee);
      setPT1(PP, TT);
    }
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"showABCD");
    if (e)
      showABCD();
  }

  void LinearTransferSystem::updateSignal() {
    s=(this->*calculateOutputMethod)();
    upds = false;
  }

  void LinearTransferSystem::updatedx() {
    dx=(A*x+B*inputSignal->evalSignal())*getStepSize();
  }

  void LinearTransferSystem::updatexd() {
    xd=A*x+B*inputSignal->evalSignal();
  }

  void LinearTransferSystem::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (inputSignalString!="")
        setInputSignal(getByPath<Signal>(inputSignalString));
    }
    else if (stage==plotting) {
      if(plotFeature[11334901831169464975ULL]==enabled and plotFeature[13300192525503281405ULL]==enabled) {
        for (int i=0; i<B.cols(); i++)
          plotColumns.push_back("input signal (" + numtostr(i) + ")");
      }
    }
    Signal::init(stage);
  }

  VecV LinearTransferSystem::outputMethodC() {
    return C*x;
  }

  VecV LinearTransferSystem::outputMethodD() {
    return D*inputSignal->evalSignal();
  }

  VecV LinearTransferSystem::outputMethodCD() {
    return outputMethodC()+outputMethodD();
  }

  void LinearTransferSystem::showABCD() {
    msg(Info)  <<  "State space modell of Object \"" << getPath() << "\":" << endl;
    msg(Info)  <<  "A Matrix:" << A << endl;
    msg(Info)  <<  "B Matrix:" << B << endl;
    msg(Info)  <<  "C Matrix:" << C << endl;
    msg(Info)  <<  "D Matrix:" << D << endl;
  }

  void LinearTransferSystem::setPID(double PP, double II, double DD) {
    if ((fabs(II)<epsroot())&&(fabs(DD)<epsroot()))
      calculateOutputMethod=&LinearTransferSystem::outputMethodD;
    else
      calculateOutputMethod=&LinearTransferSystem::outputMethodCD;

    if (fabs(DD)<epsroot()) {
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

  void LinearTransferSystem::setABCD(Mat A_, Mat B_, Mat C_, Mat D_) {
    A=A_;
    B=B_;
    C=C_;
    D=D_;
    calculateOutputMethod=&LinearTransferSystem::outputMethodCD;
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
    calculateOutputMethod=&LinearTransferSystem::outputMethodCD;
  }

  void LinearTransferSystem::setI2(double OutputGain) {
    A.resize(2, 2, INIT, 0);
    A(0,1)=1.;
    B.resize(2, 1, INIT, 0);
    B(1,0)=1.;
    C.resize(1, 2, INIT, 0);
    C(0,0)=OutputGain;
    D.resize(1, 1, INIT, 0);
    calculateOutputMethod=&LinearTransferSystem::outputMethodCD;
  }

  void LinearTransferSystem::setPT1(double P,double T) {
    A.resize(1, 1, INIT, -1./T);
    B.resize(1, 1, INIT, 1.);
    C.resize(1, 1, INIT, P/T);
    D.resize(1, 1, INIT, 0);
    calculateOutputMethod=&LinearTransferSystem::outputMethodCD;
  }

  void LinearTransferSystem::setGain(double P) {
    setPID(P, 0, 0);
  }

  void LinearTransferSystem::plot() {
    if(plotFeature[11334901831169464975ULL]==enabled and plotFeature[13300192525503281405ULL]==enabled) {
      for (int i=0; i<B.cols(); i++)
        plotVector.push_back(inputSignal->evalSignal()(i));
    }
    Signal::plot();
  }

}
