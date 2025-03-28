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
      for(auto & i : signalString)
        addInputSignal(getByPath<Signal>(i));
      if(signal.empty())
        throwError("No input signal is given!");
    }
    Signal::init(stage, config);
  }

  void Multiplexer::updateSignal() {
    int k=0;
    for (auto & i : signal) {
      const VecV &si = i->evalSignal();
      s.set(RangeV(k,k+si.size()-1),si);
      k+=si.size();;
    }
    upds = false;
  }

  int Multiplexer::getSignalSize() const {
    int size = 0;
    for (auto i : signal)
      size += i->getSignalSize();
    return size;
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, Demultiplexer)

  void Demultiplexer::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
    Vec indices = E(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMCONTROL%"indices"))->getText<Vec>();
    index.resize(indices.size());
    for(unsigned int i=0; i<index.size(); i++)
      index[i] = static_cast<Index>(indices(i))-1;
  }

  void Demultiplexer::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not signalString.empty())
        setInputSignal(getByPath<Signal>(signalString));
      if(not signal)
        throwError("Input signal is not given!");
    }
    Signal::init(stage, config);
  }

  void Demultiplexer::updateSignal() {
    const VecV &sIn = signal->evalSignal();
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
      if (not signalString.empty())
        setInputSignal(getByPath<Signal>(signalString));
      if(not s)
        throwError("Input signal is not given!");
      Signal::init(stage, config);
    }
    else
      Signal::init(stage, config);
  }

  void SignalTimeDiscretization::updateSignal() {
    if (fabs(tOld-getTime())>epsroot) {
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
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"multiplexInputSignals");
    if(e) setMultiplexInputSignals(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"function");
    if(signalString.size()==1 or multiplex)
      setFunction(ObjectFactory::createAndInit<MBSim::Function<VecV(VecV)>>(e->getFirstElementChild()));
    else if(signalString.size()==2)
      setFunction(ObjectFactory::createAndInit<MBSim::Function<VecV(VecV,VecV)>>(e->getFirstElementChild()));
    else
      throwError("(SignalOperation::initializeUsingXML): feature \"multiplex input signals\" must be enabled, if there are more than two input signals.");
  }

  void SignalOperation::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      for(auto & i : signalString)
        addInputSignal(getByPath<Signal>(i));
      if(signal.empty())
        throwError("(SignalOperation::init): no input signal is given!");
      if(signal.size()==1 or multiplex)
        updateSignal_ = &SignalOperation::updateSignal1;
      else if(signal.size()==2)
        updateSignal_ = &SignalOperation::updateSignal2;
      else
        throwError("(SignalOperation::init): number of input signals must be 1 or 2");
    }
    else if(stage==unknownStage) {
      if(f1) {
	int size = 0;
	for(size_t i=0; i<signal.size(); i++)
	  size += signal[i]->getSignalSize();
	if(f1->getArgSize()!=size) throwError(string("(SignalOperation::init): size of input signal does not match argument size of function. Size of") + (multiplex?" multiplexed ":" ") + "input signal is " + to_string(size) + ", size of argument is " + to_string(f1->getArgSize()));
      }
      else if(f2 and (f2->getArg1Size()!=signal[0]->getSignalSize() or f2->getArg2Size()!=signal[1]->getSignalSize())) throwError("(SignalOperation::init): size of input signal does not match argument size of function. Size of first input signal is " + to_string(signal[0]->getSignalSize()) + ", size of first argument is " + to_string(f2->getArg1Size()) + ". Size of second input signal is " + to_string(signal[1]->getSignalSize()) + ", size of second argument is " + to_string(f2->getArg2Size()));
    }
    Signal::init(stage, config);
    if(f1) f1->init(stage, config);
    if(f2) f2->init(stage, config);
  }

  void SignalOperation::updateSignal1() {
    if(multiplex) {
      VecV x(f1->getArgSize(),NONINIT);
      for(size_t i=0, k=0; i<signal.size(); i++) {
	VecV y = signal[i]->evalSignal();
	for(int j=0; j<signal[i]->getSignalSize(); j++, k++)
	  x(k) = y(j);
      }
      s = (*f1)(x);
    }
    else
      s = (*f1)(signal[0]->evalSignal());
    upds = false;
  }

  void SignalOperation::updateSignal2() {
    s = (*f2)(signal[0]->evalSignal(),signal[1]->evalSignal());
    upds = false;
  }

}
