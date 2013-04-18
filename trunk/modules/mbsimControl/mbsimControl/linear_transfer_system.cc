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
#include "mbsimControl/linear_transfer_system.h"
#include "mbsimControl/objectfactory.h"
#include "mbsimControl/signal_.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimControl {

  LinearTransferSystem::LinearTransferSystem(const string& name) : SignalProcessingSystem(name), R1(.002), R2(1.), c(1.) {
  }

  void LinearTransferSystem::initializeUsingXML(TiXmlElement * element) {
    SignalProcessingSystem::initializeUsingXML(element);
    TiXmlElement * e;
    TiXmlElement * ee;
    e=element->FirstChildElement(MBSIMCONTROLNS"pidType");
    if (e) {
      ee=e->FirstChildElement(MBSIMCONTROLNS"P");
      double p=Element::getDouble(ee);
      ee=e->FirstChildElement(MBSIMCONTROLNS"I");
      double i=Element::getDouble(ee);
      ee=e->FirstChildElement(MBSIMCONTROLNS"D");
      double d=Element::getDouble(ee);
      setPID(p, i, d);
    }
    e=element->FirstChildElement(MBSIMCONTROLNS"abcdType");
    if (e) {
      ee=e->FirstChildElement(MBSIMCONTROLNS"A");
      Mat AA=Element::getMat(ee);
      ee=e->FirstChildElement(MBSIMCONTROLNS"B");
      Mat BB=Element::getMat(ee, A.rows(), 0);
      ee=e->FirstChildElement(MBSIMCONTROLNS"C");
      Mat CC=Element::getMat(ee, 0, A.cols());
      ee=e->FirstChildElement(MBSIMCONTROLNS"D");
      Mat DD=Element::getMat(ee, C.rows(), B.cols());
      setABCD(AA, BB, CC, DD);
    }
    e=element->FirstChildElement(MBSIMCONTROLNS"integratorType");
    if (e) {
      ee=e->FirstChildElement(MBSIMCONTROLNS"gain");
      double g=Element::getDouble(ee);
      setIntegrator(g);
    }
    e=element->FirstChildElement(MBSIMCONTROLNS"pt1Type");
    if (e) {
      ee=e->FirstChildElement(MBSIMCONTROLNS"P");
      double PP=Element::getDouble(ee);
      ee=e->FirstChildElement(MBSIMCONTROLNS"T");
      double TT=Element::getDouble(ee);
      setPT1(PP, TT);
    }
    e=element->FirstChildElement(MBSIMCONTROLNS"showABCD");
    if (e)
      showABCD();
  }

  void LinearTransferSystem::updatedx(double t, double dt) {
    xd=(A*x+B*inputSignal->getSignal())*dt;
  }

  void LinearTransferSystem::updatexd(double t) {
    xd=A*x+B*inputSignal->getSignal();
  }

  void LinearTransferSystem::init(MBSim::InitStage stage) {
    if (stage==MBSim::resize) {
      SignalProcessingSystem::init(stage);
      x.resize(xSize, INIT, 0);
    }
    else if (stage==MBSim::plot) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        if (getPlotFeature(globalPosition)==enabled)
          for (int i=0; i<C.rows(); i++)
            plotColumns.push_back("Output Sigout (" + numtostr(i) + ")");
        if (getPlotFeature(rightHandSide)==enabled)
          for (int i=0; i<B.cols(); i++)
            plotColumns.push_back("Input Signal (" + numtostr(i) + ")");
        SignalProcessingSystem::init(stage);
      }
    }
    else
      SignalProcessingSystem::init(stage);
  }

  Vec LinearTransferSystem::calculateOutput() {
    return (this->*calculateOutputMethod)();
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

  void LinearTransferSystem::plot(double t, double dt) {
    Vec y=calculateOutput();
    if(getPlotFeature(plotRecursive)==enabled) {
      if (getPlotFeature(globalPosition)==enabled)
        for (int i=0; i<y.size(); i++)
          plotVector.push_back(y(i));
      if (getPlotFeature(rightHandSide)==enabled)
        for (int i=0; i<B.cols(); i++)
          plotVector.push_back(inputSignal->getSignal()(i));
    }
    SignalProcessingSystem::plot(t,dt);
  }

}
