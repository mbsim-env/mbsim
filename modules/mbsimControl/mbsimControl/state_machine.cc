/* Copyright (C) 2004-2022 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */


#include <config.h>
#include "mbsimControl/state_machine.h"
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, StateMachine)

  StateMachine::Transition& StateMachine::addTransition(const string &src, const string &dest, Signal *sig, double s0, int dir) {
    size_t j;
    for(j=0; j<state.size(); j++) {
      if(state[j].name==dest)
	break;
    }
    if(j==state.size())
      throwError("(StateMachine::addTransition): source not found.");
    for(auto & i : state) {
      if(i.name==src) {
	i.trans.emplace_back(Transition(j,sig,s0,dir));
	return i.trans[i.trans.size()-1];
      }
    }
    throwError("(StateMachine::addTransition): destination not found.");
  }

  void StateMachine::setInitialState(const std::string &name) {
    curis(0) = -1;
    for(size_t j=0; j<state.size(); j++) {
      if(state[j].name==name) {
	curis(0) = j;
	return;
      }
    }
    throwError("(StateMachine::setInitialState): state not found.");
  }

  void StateMachine::calcsvSize() {
    svSize = 1;
    if(rf) {
      for(auto & i : state)
	svSize = max(svSize,int(i.trans.size()));
    }
  }

  void StateMachine::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"state");
    while(e && E(e)->getTagName()==MBSIMCONTROL%"state") {
      string name = E(e)->getAttribute("name");
      double val = stod(E(e)->getAttribute("value"));
      addState(name,val);
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"transition");
    while(e && E(e)->getTagName()==MBSIMCONTROL%"transition") {
      string name = E(e)->getAttribute("source");
      string dest = E(e)->getAttribute("destination");
      double s0=0;
      int dir=0;
      if(E(e)->hasAttribute("threshold"))
	s0 = stod(E(e)->getAttribute("threshold"));
      if(E(e)->hasAttribute("direction"))
	dir = stoi(E(e)->getAttribute("direction"));
      Transition &t = addTransition(name,dest,nullptr,s0,dir);
      t.signalStr = E(e)->getAttribute("signal");
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"initialState");
    if(e)
      stateStr = E(e)->getText<string>();
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"rootFinding");
    if(e) setRootFinding(E(e)->getText<bool>());
  }

  void StateMachine::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      for(auto & i : state) {
	for(auto & j : i.trans) {
	  if(not j.signalStr.empty())
	    j.sig = getByPath<Signal>(j.signalStr);
	}
      }
    }
    else if(stage==unknownStage) {
      if(not stateStr.empty())
	setInitialState(stateStr.substr(1,stateStr.size()-2));
      nextis(0) = curis(0);
    }
    Signal::init(stage, config);
  }

  void StateMachine::aboutToUpdateInternalState() {
    for(auto & tran : state[int(curis(0))].trans)
      tran.oldSig = tran.newSig;
    if(int(nextis(0)) != int(curis(0))) {
      for(auto & tran : state[int(nextis(0))].trans)
	tran.oldSig = tran.newSig;
    }
  }

  void StateMachine::updateSignal() {
    if(rf)
      s(0) = getActiveState().val;
    else {
      nextis(0) = curis(0);
      for(auto & tran : state[int(curis(0))].trans) {
        double svo = tran.oldSig - tran.s0;
        double svn = tran.sig->evalSignal()(0) - tran.s0;
        int dir = tran.dir;
        tran.newSig = tran.oldSig;
        if(svn*svo<0) {
          tran.newSig = tran.sig->evalSignal()(0);
          if(dir==0 or (dir==1 and svn>0) or (dir==-1 and svn<0)) {
            nextis(0) = tran.dest;
            state[int(nextis(0))].t0 = getTime();
            for(auto & tran : state[int(nextis(0))].trans)
              tran.newSig = tran.sig->evalSignal()(0);
          }
        }
      }
      s(0) = state[int(nextis(0))].val;
    }
    upds = false;
  }

  void StateMachine::updateStopVector() {
    if(rf) {
      for(size_t i=0; i<getActiveState().trans.size(); i++)
        sv(i) = getActiveState().trans[i].sig->evalSignal()(0) - getActiveState().trans[i].s0;
      for(size_t i=getActiveState().trans.size(); i<size_t(sv.size()); i++)
        sv(i) = 1;
    }
    else
      sv(0) = 1;
  }

  void StateMachine::checkActive(int j) {
    if(j==1) {
      nextis(0) = curis(0);
      state[int(nextis(0))].t0 = getTime();
      for(auto & tran : state[int(nextis(0))].trans)
        tran.oldSig = tran.sig->evalSignal()(0);
    }
    else if(j==5) {
      for(int i=0; i<jsv.size(); i++) {
        if(jsv(i)) {
          int dir = getActiveState().trans[i].dir;
          double val = getActiveState().trans[i].sig->evalSignal()(0);
          double s0 = getActiveState().trans[i].s0;
          if(dir==0 or (dir==1 and val>s0) or (dir==-1 and val<s0)) {
            nextis(0) = getActiveState().trans[i].dest;
            state[int(nextis(0))].t0 = getTime();
          }
        }
      }
    }
  }

}
