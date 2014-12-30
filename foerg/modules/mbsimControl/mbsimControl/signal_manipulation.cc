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
 * Contact: markus.ms.schneider@gmail.com
 */

#include <config.h>
#include "mbsimControl/signal_manipulation.h"
#include "mbsim/utils/eps.h"
#include <fmatvec/function.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SignalMux, MBSIMCONTROL%"SignalMux")

  void SignalMux::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    while (e && E(e)->getTagName()==MBSIMCONTROL%"inputSignal") {
      string s=E(e)->getAttribute("ref");
      signalString.push_back(s);
      e=e->getNextElementSibling();
    }
  }

  void SignalMux::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      for (unsigned int i=0; i<signalString.size(); i++)
        addInputSignal(getByPath<Signal>(signalString[i]));
      signalString.clear();
      Signal::init(stage);
    }
    else if(stage==preInit) {
      Signal::init(stage);
      for(unsigned int i=0; i<signals.size(); i++)
        addDependency(signals[i]);
    }
    else
      Signal::init(stage);
  }

  void SignalMux::updateStateDependentVariables(double t) {
    VecV y=signals[0]->getSignal();
    for (unsigned int i=1; i<signals.size(); i++) {
      VecV s1=y;
      VecV s2=signals[i]->getSignal();
      y.resize(s1.size()+s2.size());
      y.set(Index(0, s1.size()-1),s1);
      y.set(Index(s1.size(), y.size()-1),s2);
    }
    s = y;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SignalDemux, MBSIMCONTROL%"SignalDemux")

  void SignalDemux::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    while (e && E(e)->getTagName()==MBSIMCONTROL%"inputSignal") {
      string s=E(e)->getAttribute("ref");
      signalString.push_back(s);
      indizesTmp.push_back(getVec(E(e)->getFirstElementChildNamed(MBSIMCONTROL%"index")));
      e=e->getNextElementSibling();
    }
  }

  void SignalDemux::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      for (unsigned int i=0; i<signalString.size(); i++) {
        Vec tmp=indizesTmp[i];
        fmatvec::VecInt tmpI(tmp.size(), INIT, 0);
        for (int j=0; j<tmp.size(); j++)
          tmpI(j)=int(tmp(j));
        addInputSignal(getByPath<Signal>(signalString[i]), tmpI);
      }
      signalString.clear();
      Signal::init(stage);
    }
    else if(stage==preInit) {
      Signal::init(stage);
      for(unsigned int i=0; i<signals.size(); i++)
        addDependency(signals[i]);
    }
    else if (stage==plotting) {
      for (unsigned int i=0; i<indizes.size(); i++)
        totalSignalSize+=indizes[i].size();
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  void SignalDemux::updateStateDependentVariables(double t) {
    VecV y(totalSignalSize, INIT, 0);
    int j=0;
    for (unsigned int i=0; i<signals.size(); i++) {
      VecV yy=signals[i]->getSignal();
      for (int k=0; k<indizes[i].size(); k++) {
        y(j)=yy(indizes[i](k));
        j++;
      }
    }
    s = y;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SignalLimitation, MBSIMCONTROL%"SignalLimitation")

  void SignalLimitation::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"minimalValue");
    VecV min=Element::getVec(e);
    setMinimalValue(min);
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"maximalValue");
    VecV max=Element::getVec(e, min.size());
    setMaximalValue(max);
  }

  void SignalLimitation::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signalString!="")
        setInputSignal(getByPath<Signal>(signalString));
      Signal::init(stage);
    }
    else if(stage==preInit) {
      Signal::init(stage);
      addDependency(s);
    }
    else
      Signal::init(stage);
  }

  void SignalLimitation::updateStateDependentVariables(double t) { 
    VecV y=s->getSignal();
    for (int i=0; i<y.size(); i++)
      if (y(i)<minValue(i))
        y(i)=minValue(i);
      else if (y(i)>maxValue(i))
        y(i)=maxValue(i);
    Signal::s = y; 
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SignalTimeDiscretization, MBSIMCONTROL%"SignalTimeDiscretization")

  void SignalTimeDiscretization::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
  }

  void SignalTimeDiscretization::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signalString!="")
        setInputSignal(getByPath<Signal>(signalString));
      Signal::init(stage);
    }
    else if(stage==preInit) {
      Signal::init(stage);
      addDependency(s);
    }
    else if(stage==resize) {
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  void SignalTimeDiscretization::updateg(double t) { 
  }

  void SignalTimeDiscretization::updateStateDependentVariables(double t) { 
    if (fabs(tOld-t)>epsroot()) {
      Signal::s=s->getSignal(); 
      tOld=t; 
    } 
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PIDController, MBSIMCONTROL%"PIDController")

  void PIDController::initializeUsingXML(DOMElement * element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    sString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"derivativeOfInputSignal");
    sdString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"P");
    double p=Element::getDouble(e);
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"I");
    double i=Element::getDouble(e);
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"D");
    double d=Element::getDouble(e);
    setPID(p, i, d);
  }

  void PIDController::updatedx(double t, double dt) {
    if(xSize) xd=s->getSignal()*dt;
  }

  void PIDController::updatexd(double t) {
    if(xSize) xd=s->getSignal();
  }

  void PIDController::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (sString!="")
        setInputSignal(getByPath<Signal>(sString));
      if (sdString!="")
        setDerivativeOfInputSignal(getByPath<Signal>(sdString));
      Signal::init(stage);
    }
    else if(stage==preInit) {
      Signal::init(stage);
      addDependency(s);
      addDependency(sd);
    }
    else if (stage==resize) {
      Signal::init(stage);
      x.resize(xSize, INIT, 0);
    }
    else if (stage==plotting) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        Signal::init(stage);
      }
    }
    else
      Signal::init(stage);
  }

  void PIDController::updateStateDependentVariables(double t) {
    (this->*updateSignalMethod)();
  }

  void PIDController::updateSignalPID() {
    Signal::s = P*s->getSignal() + D*sd->getSignal() + I*x;
  }

  void PIDController::updateSignalPD() {
    Signal::s = P*s->getSignal() + D*sd->getSignal();
  }

  void PIDController::setPID(double PP, double II, double DD) {
    if ((fabs(II)<epsroot()))
      updateSignalMethod=&PIDController::updateSignalPD;
    else
      updateSignalMethod=&PIDController::updateSignalPID;
    P = PP; I = II; D = DD;
  }

  void PIDController::plot(double t, double dt) {
    Signal::plot(t,dt);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(UnarySignalOperation, MBSIMCONTROL%"UnarySignalOperation")

  void UnarySignalOperation::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"function");
    if(e) {
      MBSim::Function<VecV(VecV)> *f=ObjectFactory::createAndInit<MBSim::Function<VecV(VecV)> >(e->getFirstElementChild());
      setFunction(f);
    }
  }

  void UnarySignalOperation::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signalString!="")
        setInputSignal(getByPath<Signal>(signalString));
      Signal::init(stage);
    }
    else if(stage==preInit) {
      Signal::init(stage);
      addDependency(s);
    }
    else
      Signal::init(stage);
    f->init(stage);
  }

  void UnarySignalOperation::updateStateDependentVariables(double t) { 
    Signal::s = (*f)(s->getSignal()); 
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(BinarySignalOperation, MBSIMCONTROL%"BinarySignalOperation")

  void BinarySignalOperation::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"firstInputSignal");
    signal1String=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"secondInputSignal");
    signal2String=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"function");
    if(e) {
      MBSim::Function<VecV(VecV,VecV)> *f=ObjectFactory::createAndInit<MBSim::Function<VecV(VecV,VecV)> >(e->getFirstElementChild());
      setFunction(f);
    }
  }

  void BinarySignalOperation::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signal1String!="")
        setFirstInputSignal(getByPath<Signal>(signal1String));
      if (signal2String!="")
        setSecondInputSignal(getByPath<Signal>(signal2String));
      Signal::init(stage);
    }
    else if(stage==preInit) {
      Signal::init(stage);
      addDependency(s1);
      addDependency(s2);
    }
    else
      Signal::init(stage);
    f->init(stage);
  }

  void BinarySignalOperation::updateStateDependentVariables(double t) { 
    s = (*f)(s1->getSignal(),s2->getSignal()); 
  }

}

