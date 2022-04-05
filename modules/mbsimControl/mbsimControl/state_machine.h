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

#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

#include "mbsimControl/signal_.h"

namespace MBSimControl {

  /*!
   * \brief State machine
   * \author Martin Foerg
   */
  class StateMachine : public Signal {
    struct Transition {
      Transition(int dest_, Signal *sig_, double s0_) : dest(dest_), sig(sig_), s0(s0_) { }
      int dest;
      Signal *sig;
      double s0;
      std::string signalStr;
    };
    struct State {
      State(const std::string &name_, double val_) : name(name_), val(val_), t0(0) { }
      void addTransition(const Transition &trans_) { trans.emplace_back(trans_); }
      std::string name;
      double val;
      double t0;
      std::vector<Transition> trans;
    };
    public:
      StateMachine(const std::string &name="") : Signal(name) { }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void addState(const std::string &name, double val) { state.emplace_back(State(name,val)); }
      Transition& addTransition(const std::string &name, const std::string &dest, Signal *sig, double s0=0);
      void setInitialState(const std::string &name);
      const State& getActiveState() const { return state[activeState]; }
      void updateSignal() override;
      int getSignalSize() const override { return 1; }
      bool isSetValued() const override { return true; }
      void calcsvSize() override;
      void updateStopVector() override;
      void checkActive(int j) override;
      bool isActive() const override { return true; }
    private:
      std::vector<State> state;
      std::vector<Transition> transition;
      int activeState{0};
  };

}

#endif
