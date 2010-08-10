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
 * Contact: schneidm@users.berlios.de
 */

#include "mbsimControl/signal_manipulation.h"
#include "mbsimControl/objectfactory.h"
#include "mbsimControl/obsolet_hint.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimControl {

  void SignalAddition::initializeUsingXML(TiXmlElement *element) {
    Signal::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMCONTROLNS"inputSignal");
    while (e && e->ValueStr()==MBSIMCONTROLNS"inputSignal") {
      string path=e->Attribute("ref");
      double f=getDouble(e->FirstChildElement(MBSIMCONTROLNS"factor"));
      signalString.push_back(path);
      factorsTmp.push_back(f);
      e=e->NextSiblingElement();
    }
  }

  void SignalAddition::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      for (unsigned int i=0; i<signalString.size(); i++)
        addSignal(getByPath<Signal>(process_signal_string(signalString[i])), factorsTmp[i]);
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

  
  void SignalOffset::initializeUsingXML(TiXmlElement *element) {
    Signal::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMCONTROLNS"inputSignal");
    signalString=e->Attribute("ref");
    setOffset(getVec(element->FirstChildElement(MBSIMCONTROLNS"offset")));
  }

  void SignalOffset::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      setSignal(getByPath<Signal>(process_signal_string(signalString)));
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  Vec SignalOffset::getSignal() {
    return signal->getSignal()+offset;
  }

  
  void SignalMultiplication::initializeUsingXML(TiXmlElement *element) {
    Signal::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMCONTROLNS"inputSignal");
    while (e && e->ValueStr()==MBSIMCONTROLNS"inputSignal") {
      signalString.push_back(e->Attribute("ref"));
      exponentsTmp.push_back(getDouble(e->FirstChildElement(MBSIMCONTROLNS"exponent")));
      e=e->NextSiblingElement();
    }
  }

  void SignalMultiplication::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      for (unsigned int i=0; i<signalString.size(); i++)
        addSignal(getByPath<Signal>(process_signal_string(signalString[i])), exponentsTmp[i]);
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


  void SignalMux::initializeUsingXML(TiXmlElement *element) {
    Signal::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMCONTROLNS"inputSignal");
    while (e && e->ValueStr()==MBSIMCONTROLNS"inputSignal") {
      string s=e->Attribute("ref");
      signalString.push_back(s);
      e=e->NextSiblingElement();
    }
  }

  void SignalMux::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      for (unsigned int i=0; i<signalString.size(); i++)
        addSignal(getByPath<Signal>(process_signal_string(signalString[i])));
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


  void SignalLimitation::initializeUsingXML(TiXmlElement *element) {
    Signal::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMCONTROLNS"inputSignal");
    signalString=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMCONTROLNS"minimalValue");
    Vec min=Element::getVec(e);
    setMinimalValue(min);
    e=element->FirstChildElement(MBSIMCONTROLNS"maximalValue");
    Vec max=Element::getVec(e, min.size());
    setMaximalValue(max);
  }

  void SignalLimitation::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signalString!="")
        setSignal(getByPath<Signal>(process_signal_string(signalString)));
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


  void SignalTimeDiscretization::initializeUsingXML(TiXmlElement *element) {
    Signal::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMCONTROLNS"inputSignal");
    signalString=e->Attribute("ref");
  }

  void SignalTimeDiscretization::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signalString!="")
        setSignal(getByPath<Signal>(process_signal_string(signalString)));
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


  void SignalOperation::initializeUsingXML(TiXmlElement *element) {
    Signal::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMCONTROLNS"inputSignal");
    signalString=e->Attribute("ref");
    e=e->NextSiblingElement();
    if (e->ValueStr()==MBSIMCONTROLNS"acos")
      setOperator(1);
    else if (e->ValueStr()==MBSIMCONTROLNS"asin")
      setOperator(2);
    else if (e->ValueStr()==MBSIMCONTROLNS"atan")
      setOperator(3);
    else if (e->ValueStr()==MBSIMCONTROLNS"atan2") {
      setOperator(4);
      TiXmlElement * ee=e->FirstChildElement();
      if (ee->ValueStr()==MBSIMCONTROLNS"input2Signal")
        signal2String=ee->Attribute("ref");
      else if (ee->ValueStr()==MBSIMCONTROLNS"yValues")
        setSecondSignalValues(Element::getVec(ee));
    }
    else if (e->ValueStr()==MBSIMCONTROLNS"ceil")
      setOperator(5);
    else if (e->ValueStr()==MBSIMCONTROLNS"cos")
      setOperator(6);
    else if (e->ValueStr()==MBSIMCONTROLNS"cosh")
      setOperator(7);
    else if (e->ValueStr()==MBSIMCONTROLNS"cbrt")
      setOperator(8);
    else if (e->ValueStr()==MBSIMCONTROLNS"exp")
      setOperator(9);
    else if (e->ValueStr()==MBSIMCONTROLNS"exp2")
      setOperator(10);
    else if (e->ValueStr()==MBSIMCONTROLNS"expm1")
      setOperator(11);
    else if (e->ValueStr()==MBSIMCONTROLNS"fabs")
      setOperator(12);
    else if (e->ValueStr()==MBSIMCONTROLNS"floor")
      setOperator(13);
    else if (e->ValueStr()==MBSIMCONTROLNS"fmod") {
      setOperator(14);
      TiXmlElement * ee=e->FirstChildElement();
      if (ee->ValueStr()==MBSIMCONTROLNS"input2Signal")
        signal2String=ee->Attribute("ref");
      else if (ee->ValueStr()==MBSIMCONTROLNS"yValues")
        setSecondSignalValues(Element::getVec(ee));
    }
    else if (e->ValueStr()==MBSIMCONTROLNS"hypot") {
      setOperator(15);
      TiXmlElement * ee=e->FirstChildElement();
      if (ee->ValueStr()==MBSIMCONTROLNS"input2Signal")
        signal2String=ee->Attribute("ref");
      else if (ee->ValueStr()==MBSIMCONTROLNS"yValues")
        setSecondSignalValues(Element::getVec(ee));
    }
    else if (e->ValueStr()==MBSIMCONTROLNS"log")
      setOperator(16);
    else if (e->ValueStr()==MBSIMCONTROLNS"log2")
      setOperator(17);
    else if (e->ValueStr()==MBSIMCONTROLNS"logb")
      setOperator(18);
    else if (e->ValueStr()==MBSIMCONTROLNS"log10")
      setOperator(19);
    else if (e->ValueStr()==MBSIMCONTROLNS"log1p")
      setOperator(20);
    else if (e->ValueStr()==MBSIMCONTROLNS"pow") {
      setOperator(21);
      TiXmlElement * ee=e->FirstChildElement();
      if (ee->ValueStr()==MBSIMCONTROLNS"input2Signal")
        signal2String=ee->Attribute("ref");
      else if (ee->ValueStr()==MBSIMCONTROLNS"yValues")
        setSecondSignalValues(Element::getVec(ee));
    }
    else if (e->ValueStr()==MBSIMCONTROLNS"sin")
      setOperator(22);
    else if (e->ValueStr()==MBSIMCONTROLNS"sinh")
      setOperator(23);
    else if (e->ValueStr()==MBSIMCONTROLNS"sqrt")
      setOperator(24);
    else if (e->ValueStr()==MBSIMCONTROLNS"tan")
      setOperator(25);
    else if (e->ValueStr()==MBSIMCONTROLNS"tanh")
      setOperator(26);
  }

  void SignalOperation::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signalString!="")
        setSignal(getByPath<Signal>(process_signal_string(signalString)));
      if (signal2String!="")
        setSecondSignal(getByPath<Signal>(process_signal_string(signal2String)));
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
  
  void SpecialSignalOperation::initializeUsingXML(TiXmlElement *element) {
    Signal::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMCONTROLNS"inputSignal");
    signalString=e->Attribute("ref");
    e=e->NextSiblingElement();
    if (e->ValueStr()==MBSIMCONTROLNS"sign")
      setOperator(1);
    else if (e->ValueStr()==MBSIMCONTROLNS"selection") {
      setOperator(2);
      TiXmlElement * ee=e->FirstChildElement();
      if (ee->ValueStr()==MBSIMCONTROLNS"input2Signal")
        signal2String=ee->Attribute("ref");
      else if (ee->ValueStr()==MBSIMCONTROLNS"yValues")
        setSecondSignalValues(Element::getVec(ee));
    }
  }

  void SpecialSignalOperation::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signalString!="")
        setSignal(getByPath<Signal>(process_signal_string(signalString)));
      if (signal2String!="")
        setSecondSignal(getByPath<Signal>(process_signal_string(signal2String)));
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

}

