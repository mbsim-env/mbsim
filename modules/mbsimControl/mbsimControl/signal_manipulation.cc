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
#include "mbsim/utils/index.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, Multiplexer)

  void Multiplexer::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    while(e && E(e)->getTagName()==MBSIMCONTROL%"inputSignal") {
      signalString.push_back(E(e)->getAttribute("ref"));
      e=e->getNextElementSibling();
    }
  }

  void Multiplexer::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      for(unsigned int i=0; i<signalString.size(); i++)
        addInputSignal(getByPath<Signal>(signalString[i]));
    }
    else if(stage==preInit)
      s.resize(getSignalSize(),NONINIT);
    Signal::init(stage, config);
  }

  void Multiplexer::updateSignal() {
    int k=0;
    for (unsigned int i=0; i<signal.size(); i++) {
      VecV si = signal[i]->evalSignal();
      s.set(RangeV(k,k+si.size()-1),si);
      k+=si.size();;
    }
    upds = false;
  }

  int Multiplexer::getSignalSize() const {
    int size = 0;
    for (unsigned int i=0; i<signal.size(); i++)
      size += signal[i]->getSignalSize();
    return size;
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, Demultiplexer)

  void Demultiplexer::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
    Vec indices = Element::getVec(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMCONTROL%"indices"));
    index.resize(indices.size());
    for(unsigned int i=0; i<index.size(); i++)
      index[i] = static_cast<Index>(indices(i))-1;
  }

  void Demultiplexer::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(signalString!="")
        setInputSignal(getByPath<Signal>(signalString));
    }
    else if(stage==preInit)
      s.resize(getSignalSize(),NONINIT);
    Signal::init(stage, config);
  }

  void Demultiplexer::updateSignal() {
    VecV sIn = signal->evalSignal();
    for (unsigned int i=0; i<index.size(); i++) {
      s(i) = sIn(index[i]);
    }
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, SignalTimeDiscretization)

  void SignalTimeDiscretization::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
  }

  void SignalTimeDiscretization::init(InitStage stage, const InitConfigSet &config) {
    if (stage==resolveStringRef) {
      if (signalString!="")
        setInputSignal(getByPath<Signal>(signalString));
      Signal::init(stage, config);
    }
    else
      Signal::init(stage, config);
  }

  void SignalTimeDiscretization::updateSignal() {
    if (fabs(tOld-getTime())>epsroot()) {
      Signal::s=s->evalSignal();
      tOld=getTime();
    }
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, SignalOperation)

  void SignalOperation::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    while(E(e)->getTagName()==MBSIMCONTROL%"inputSignal") {
      signalString.push_back(E(e)->getAttribute("ref"));
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"function");
    if(signalString.size()==1)
      setFunction(ObjectFactory::createAndInit<MBSim::Function<VecV(VecV)> >(e->getFirstElementChild()));
    else if(signalString.size()==2)
      setFunction(ObjectFactory::createAndInit<MBSim::Function<VecV(VecV,VecV)> >(e->getFirstElementChild()));
  }

  void SignalOperation::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      for(unsigned int i=0; i<signalString.size(); i++)
        addInputSignal(getByPath<Signal>(signalString[i]));
      if(signal.size()==1)
        updateSignal_ = &SignalOperation::updateSignal1;
      else if(signal.size()==2)
        updateSignal_ = &SignalOperation::updateSignal2;
      else
        THROW_MBSIMERROR("(SignalOperation::init): number of input signals must be 1 or 2");
    }
    Signal::init(stage, config);
    if(f1) f1->init(stage, config);
    if(f2) f2->init(stage, config);
  }

  void SignalOperation::updateSignal1() {
    s = (*f1)(signal[0]->evalSignal());
    upds = false;
  }

  void SignalOperation::updateSignal2() {
    s = (*f2)(signal[0]->evalSignal(),signal[1]->evalSignal());
    upds = false;
  }

}
