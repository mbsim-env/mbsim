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
#include "mbsimControl/defines.h"
#include "mbsim/utils/eps.h"
#include <fmatvec/function.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalAddition, MBSIMCONTROL%"SignalAddition")

  void SignalAddition::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    while (e && E(e)->getTagName()==MBSIMCONTROL%"inputSignal") {
      string path=E(e)->getAttribute("ref");
      double f=getDouble(E(e)->getFirstElementChildNamed(MBSIMCONTROL%"factor"));
      signalString.push_back(path);
      factorsTmp.push_back(f);
      e=e->getNextElementSibling();
    }
  }

  void SignalAddition::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      for (unsigned int i=0; i<signalString.size(); i++)
        addSignal(getByPath<Signal>(signalString[i]), factorsTmp[i]);
      signalString.clear();
      factorsTmp.clear();
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  void SignalAddition::addSignal(Signal * signal, double factor) {
    signals.push_back(signal);
    factors.push_back(factor);
  }

  Vec SignalAddition::getSignal() {
    Vec y=factors[0]*(signals[0]->getSignal());
    for (unsigned int i=1; i<signals.size(); i++)
      y+=factors[i]*(signals[i]->getSignal());
    return y;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalOffset, MBSIMCONTROL%"SignalOffset")
  
  void SignalOffset::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
    setOffset(getVec(E(element)->getFirstElementChildNamed(MBSIMCONTROL%"offset")));
  }

  void SignalOffset::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      setSignal(getByPath<Signal>(signalString));
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  Vec SignalOffset::getSignal() {
    return signal->getSignal()+offset;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalMultiplication, MBSIMCONTROL%"SignalMultiplication")
  
  void SignalMultiplication::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    while (e && E(e)->getTagName()==MBSIMCONTROL%"inputSignal") {
      signalString.push_back(E(e)->getAttribute("ref"));
      exponentsTmp.push_back(getDouble(E(e)->getFirstElementChildNamed(MBSIMCONTROL%"exponent")));
      e=e->getNextElementSibling();
    }
  }

  void SignalMultiplication::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      for (unsigned int i=0; i<signalString.size(); i++)
        addSignal(getByPath<Signal>(signalString[i]), exponentsTmp[i]);
      signalString.clear();
      exponentsTmp.clear();
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  void SignalMultiplication::addSignal(Signal * signal, double exp) {
    signals.push_back(signal);
    exponents.push_back(exp);
  }

  Vec SignalMultiplication::getSignal() {
    Vec y=signals[0]->getSignal();
    for (int i=0; i<y.size(); i++)
      y(i)=pow(y(i), exponents[0]);
    for (unsigned int i=1; i<signals.size(); i++) {
      const Vec y2=signals[i]->getSignal();
      for (int j=0; j<y.size(); j++)
        y(j)*=pow(y2(j), exponents[i]);
    }
    return y;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalMux, MBSIMCONTROL%"SignalMux")

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
        addSignal(getByPath<Signal>(signalString[i]));
      signalString.clear();
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  Vec SignalMux::getSignal() {
    Vec y=signals[0]->getSignal();
    for (unsigned int i=1; i<signals.size(); i++) {
      Vec s1=y;
      Vec s2=signals[i]->getSignal();
      y.resize(s1.size()+s2.size());
      y(Index(0, s1.size()-1))=s1;
      y(Index(s1.size(), y.size()-1))=s2;
    }
    return y;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalDemux, MBSIMCONTROL%"SignalDemux")

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
        addSignal(getByPath<Signal>(signalString[i]), tmpI);
      }
      signalString.clear();
      Signal::init(stage);
    }
    else if (stage==MBSim::plot) {
      for (unsigned int i=0; i<indizes.size(); i++)
        totalSignalSize+=indizes[i].size();
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  Vec SignalDemux::getSignal() {
    Vec y(totalSignalSize, INIT, 0);
    int j=0;
    for (unsigned int i=0; i<signals.size(); i++) {
      Vec yy=signals[i]->getSignal().copy();
      for (int k=0; k<indizes[i].size(); k++) {
        y(j)=yy(indizes[i](k));
        j++;
      }
    }
    return y;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalLimitation, MBSIMCONTROL%"SignalLimitation")

  void SignalLimitation::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"minimalValue");
    Vec min=Element::getVec(e);
    setMinimalValue(min);
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"maximalValue");
    Vec max=Element::getVec(e, min.size());
    setMaximalValue(max);
  }

  void SignalLimitation::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signalString!="")
        setSignal(getByPath<Signal>(signalString));
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  Vec SignalLimitation::getSignal() { 
    Vec y=s->getSignal();
    for (int i=0; i<y.size(); i++)
      if (y(i)<minValue(i))
        y(i)=minValue(i);
      else if (y(i)>maxValue(i))
        y(i)=maxValue(i);
    return y; 
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalTimeDiscretization, MBSIMCONTROL%"SignalTimeDiscretization")

  void SignalTimeDiscretization::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
  }

  void SignalTimeDiscretization::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signalString!="")
        setSignal(getByPath<Signal>(signalString));
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  void SignalTimeDiscretization::updateg(double t) { 
    if (fabs(tOld-t)>epsroot()) {
      y=s->getSignal(); 
      tOld=t; 
    } 
  }

  Vec SignalTimeDiscretization::getSignal() { 
    if (y.size()==0)
      updateg(-98e99);
    return y; 
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalOperation, MBSIMCONTROL%"SignalOperation")

  void SignalOperation::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
    e=e->getNextElementSibling();
    if (E(e)->getTagName()==MBSIMCONTROL%"acos")
      setOperator(1);
    else if (E(e)->getTagName()==MBSIMCONTROL%"asin")
      setOperator(2);
    else if (E(e)->getTagName()==MBSIMCONTROL%"atan")
      setOperator(3);
    else if (E(e)->getTagName()==MBSIMCONTROL%"atan2") {
      setOperator(4);
      DOMElement * ee=e->getFirstElementChild();
      if (E(ee)->getTagName()==MBSIMCONTROL%"input2Signal")
        signal2String=E(ee)->getAttribute("ref");
      else if (E(ee)->getTagName()==MBSIMCONTROL%"yValues")
        setSecondSignalValues(Element::getVec(ee));
    }
    else if (E(e)->getTagName()==MBSIMCONTROL%"ceil")
      setOperator(5);
    else if (E(e)->getTagName()==MBSIMCONTROL%"cos")
      setOperator(6);
    else if (E(e)->getTagName()==MBSIMCONTROL%"cosh")
      setOperator(7);
    else if (E(e)->getTagName()==MBSIMCONTROL%"cbrt")
      setOperator(8);
    else if (E(e)->getTagName()==MBSIMCONTROL%"exp")
      setOperator(9);
    else if (E(e)->getTagName()==MBSIMCONTROL%"exp2")
      setOperator(10);
    else if (E(e)->getTagName()==MBSIMCONTROL%"expm1")
      setOperator(11);
    else if (E(e)->getTagName()==MBSIMCONTROL%"fabs")
      setOperator(12);
    else if (E(e)->getTagName()==MBSIMCONTROL%"floor")
      setOperator(13);
    else if (E(e)->getTagName()==MBSIMCONTROL%"fmod") {
      setOperator(14);
      DOMElement * ee=e->getFirstElementChild();
      if (E(ee)->getTagName()==MBSIMCONTROL%"input2Signal")
        signal2String=E(ee)->getAttribute("ref");
      else if (E(ee)->getTagName()==MBSIMCONTROL%"yValues")
        setSecondSignalValues(Element::getVec(ee));
    }
    else if (E(e)->getTagName()==MBSIMCONTROL%"hypot") {
      setOperator(15);
      DOMElement * ee=e->getFirstElementChild();
      if (E(ee)->getTagName()==MBSIMCONTROL%"input2Signal")
        signal2String=E(ee)->getAttribute("ref");
      else if (E(ee)->getTagName()==MBSIMCONTROL%"yValues")
        setSecondSignalValues(Element::getVec(ee));
    }
    else if (E(e)->getTagName()==MBSIMCONTROL%"log")
      setOperator(16);
    else if (E(e)->getTagName()==MBSIMCONTROL%"log2")
      setOperator(17);
    else if (E(e)->getTagName()==MBSIMCONTROL%"logb")
      setOperator(18);
    else if (E(e)->getTagName()==MBSIMCONTROL%"log10")
      setOperator(19);
    else if (E(e)->getTagName()==MBSIMCONTROL%"log1p")
      setOperator(20);
    else if (E(e)->getTagName()==MBSIMCONTROL%"pow") {
      setOperator(21);
      DOMElement * ee=e->getFirstElementChild();
      if (E(ee)->getTagName()==MBSIMCONTROL%"input2Signal")
        signal2String=E(ee)->getAttribute("ref");
      else if (E(ee)->getTagName()==MBSIMCONTROL%"yValues")
        setSecondSignalValues(Element::getVec(ee));
    }
    else if (E(e)->getTagName()==MBSIMCONTROL%"sin")
      setOperator(22);
    else if (E(e)->getTagName()==MBSIMCONTROL%"sinh")
      setOperator(23);
    else if (E(e)->getTagName()==MBSIMCONTROL%"sqrt")
      setOperator(24);
    else if (E(e)->getTagName()==MBSIMCONTROL%"tan")
      setOperator(25);
    else if (E(e)->getTagName()==MBSIMCONTROL%"tanh")
      setOperator(26);
  }

  void SignalOperation::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signalString!="")
        setSignal(getByPath<Signal>(signalString));
      if (signal2String!="")
        setSecondSignal(getByPath<Signal>(signal2String));
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  Vec SignalOperation::getSignal() { 
    Vec y=s->getSignal().copy();
    Vec y2=s2?s2->getSignal().copy():s2values;
    if (op==1)
      for (int i=0; i<y.size(); i++)
        y(i)=acos((fabs(y(i))>=1)?((y(i)>0)?1.:-1.):y(i));
    else if (op==2)
      for (int i=0; i<y.size(); i++)
        y(i)=asin((fabs(y(i))>=1)?((y(i)>0)?1.:-1.):y(i));
    else if (op==3)
      for (int i=0; i<y.size(); i++)
        y(i)=atan(y(i));
    else if (op==4)
      for (int i=0; i<y.size(); i++)
        y(i)=atan2(y(i), y2(i));
    else if (op==5)
      for (int i=0; i<y.size(); i++)
        y(i)=ceil(y(i));
    else if (op==6)
      for (int i=0; i<y.size(); i++)
        y(i)=cos(y(i));
    else if (op==7)
      for (int i=0; i<y.size(); i++)
        y(i)=cosh(y(i));
    else if (op==8)
      for (int i=0; i<y.size(); i++)
        y(i)=cbrt(y(i));
    else if (op==9)
      for (int i=0; i<y.size(); i++)
        y(i)=exp(y(i));
    else if (op==10)
      for (int i=0; i<y.size(); i++)
        y(i)=exp2(y(i));
    else if (op==11)
      for (int i=0; i<y.size(); i++)
        y(i)=expm1(y(i));
    else if (op==12)
      for (int i=0; i<y.size(); i++)
        y(i)=fabs(y(i));
    else if (op==13)
      for (int i=0; i<y.size(); i++)
        y(i)=floor(y(i));
    else if (op==14)
      for (int i=0; i<y.size(); i++)
        y(i)=fmod(y(i), y2(i));
    else if (op==15)
      for (int i=0; i<y.size(); i++)
        y(i)=hypot(y(i), y2(i));
    else if (op==16)
      for (int i=0; i<y.size(); i++)
        y(i)=log(fabs(y(i)));
    else if (op==17)
      for (int i=0; i<y.size(); i++)
        y(i)=log2(fabs(y(i)));
    else if (op==18)
      for (int i=0; i<y.size(); i++)
        y(i)=logb(y(i));
    else if (op==19)
      for (int i=0; i<y.size(); i++)
        y(i)=log10(fabs(y(i)));
    else if (op==20)
      for (int i=0; i<y.size(); i++) 
        y(i)=log1p(((y(i)<=-1.)?-1.:y(i)));
    else if (op==21)
      for (int i=0; i<y.size(); i++)
        y(i)=pow(y(i), y2(i));
    else if (op==22)
      for (int i=0; i<y.size(); i++)
        y(i)=sin(y(i));
    else if (op==23)
      for (int i=0; i<y.size(); i++)
        y(i)=sinh(y(i));
    else if (op==24)
      for (int i=0; i<y.size(); i++)
        y(i)=sqrt(fabs(y(i)));
    else if (op==25)
      for (int i=0; i<y.size(); i++)
        y(i)=tan(y(i));
    else if (op==26)
      for (int i=0; i<y.size(); i++)
        y(i)=tanh(y(i));
    return y; 
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SpecialSignalOperation, MBSIMCONTROL%"SpecialSignalOperation")
  
  void SpecialSignalOperation::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
    e=e->getNextElementSibling();
    if (E(e)->getTagName()==MBSIMCONTROL%"sign")
      setOperator(1);
    else if (E(e)->getTagName()==MBSIMCONTROL%"selection") {
      setOperator(2);
      DOMElement * ee=e->getFirstElementChild();
      if (E(ee)->getTagName()==MBSIMCONTROL%"input2Signal")
        signal2String=E(ee)->getAttribute("ref");
      else if (E(ee)->getTagName()==MBSIMCONTROL%"yValues")
        setSecondSignalValues(Element::getVec(ee));
    }
  }

  void SpecialSignalOperation::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signalString!="")
        setSignal(getByPath<Signal>(signalString));
      if (signal2String!="")
        setSecondSignal(getByPath<Signal>(signal2String));
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  Vec SpecialSignalOperation::getSignal() { 
    Vec y=s->getSignal();
    Vec y2=s2?s2->getSignal():s2values;
    if (op==1)
      for (int i=0; i<y.size(); i++)
        y(i)=(fabs(y(i))<macheps())?0:((y(i)>0)?1.:-1.);
    else if (op==2)
      for (int i=0; i<y.size(); i++)
        y(i)=y2(i)>0?y(i):0;
    return y; 
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, PIDController, MBSIMCONTROL%"PIDController")

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
    xd=s->getSignal()*dt;
  }

  void PIDController::updatexd(double t) {
    xd=s->getSignal();
  }

  void PIDController::init(MBSim::InitStage stage) {
    if (stage==resolveXMLPath) {
      if (sString!="")
        setInputSignal(getByPath<Signal>(sString));
      if (sdString!="")
        setDerivativeOfInputSignal(getByPath<Signal>(sdString));
      Signal::init(stage);
    }
    else if (stage==MBSim::resize) {
      Signal::init(stage);
      x.resize(xSize, INIT, 0);
    }
    else if (stage==MBSim::plot) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        Signal::init(stage);
      }
    }
    else
      Signal::init(stage);
  }

  Vec PIDController::getSignal() {
    return (this->*getSignalMethod)();
  }

  Vec PIDController::getSignalPID() {
    return P*s->getSignal() + D*sd->getSignal() + I*x;
  }

  Vec PIDController::getSignalPD() {
    return P*s->getSignal() + D*sd->getSignal();
  }

  void PIDController::setPID(double PP, double II, double DD) {
    if ((fabs(II)<epsroot()))
      getSignalMethod=&PIDController::getSignalPD;
    else
      getSignalMethod=&PIDController::getSignalPID;
    P = PP; I = II; D = DD;
  }

  void PIDController::plot(double t, double dt) {
    Signal::plot(t,dt);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, UnarySignalOperation, MBSIMCONTROL%"UnarySignalOperation")

  void UnarySignalOperation::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"function");
    if(e) {
      fmatvec::Function<Vec(Vec)> *f=ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Vec(Vec)> >(e->getFirstElementChild());
      setFunction(f);
    }
  }

  void UnarySignalOperation::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signalString!="")
        setSignal(getByPath<Signal>(signalString));
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  Vec UnarySignalOperation::getSignal() { 
    return (*f)(s->getSignal()); 
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, BinarySignalOperation, MBSIMCONTROL%"BinarySignalOperation")

  void BinarySignalOperation::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"input1Signal");
    signal1String=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"input2Signal");
    signal2String=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"function");
    if(e) {
      fmatvec::Function<Vec(Vec,Vec)> *f=ObjectFactory<fmatvec::FunctionBase>::createAndInit<fmatvec::Function<Vec(Vec,Vec)> >(e->getFirstElementChild());
      setFunction(f);
    }
  }

  void BinarySignalOperation::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signal1String!="")
        setSignal1(getByPath<Signal>(signal1String));
      if (signal2String!="")
        setSignal2(getByPath<Signal>(signal2String));
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  Vec BinarySignalOperation::getSignal() { 
    return (*f)(s1->getSignal(),s2->getSignal()); 
  }

}

