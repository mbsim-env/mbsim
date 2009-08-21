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

#ifndef _SIGNAL_MANIPULATION_H_
#define _SIGNAL_MANIPULATION_H_

#include "mbsimControl/signal_.h"

namespace MBSim {

  class SignalAddition : public Signal {
    public:
      SignalAddition(const std::string &name) : Signal(name) {}
      
      void addSignal(Signal * signal, double factor=1.) {
        signals.push_back(signal);
        factors.push_back(factor);
      }
      
      fmatvec::Vec getSignal() {
        signal=factors[0]*(signals[0]->getSignal());
        for (unsigned int i=1; i<signals.size(); i++)
          signal+=factors[i]*(signals[i]->getSignal());
        return signal;
      }
      
      void init() {
        Signal::init();
        signal.resize(1); //TODO
//        signal.resize((signals[0]->getSignal()).size());
//        for (unsigned int i=1; i<signals.size(); i++)
//          assert((signals[i]->getSignal()).size()==signal.size());
      }
    
    private:
      std::vector<Signal *> signals;
      std::vector<double> factors;
  };


  class SignalMux : public Signal {  
    public:
      SignalMux(const std::string &name) : Signal(name) {}
      
      void addSignal(Signal * signal) {signals.push_back(signal); }
      
      void init() {
        Signal::init();
        signalIndex.push_back((signals[0]->getSignal()).size()-1);
        for (unsigned int i=1; i<signals.size(); i++)
          signalIndex[i]=signalIndex[i-1]+(signals[i]->getSignal()).size();
        signal.resize(signalIndex.back()+1);
      }
      
      fmatvec::Vec getSignal() {
        signal(fmatvec::Index(0, signalIndex[0]))=signals[0]->getSignal();
        for (unsigned int i=1; i<signals.size(); i++)
          signal(fmatvec::Index(signalIndex[i-1]+1, signalIndex[i]))=signals[i]->getSignal();
        return signal;
      }
    
    private:
      std::vector<Signal *> signals;
      std::vector<int> signalIndex;
  };


  class SignalTimeDiscretization : public Signal {  
    public:
      SignalTimeDiscretization(const std::string &name) : Signal(name), tOld(-99e99) {}
      
      void setSignal(Signal * signal_) {s=signal_; }
      
      void init() {
        Signal::init();
        signal.resize(1);
      }

      void updateg(double t) {
        if (tOld!=t) {
          signal=s->getSignal();
          tOld=t;
        }
      }
      
      fmatvec::Vec getSignal() { return signal; }
    
    private:
      Signal * s;
      double tOld;
  };


}

#endif /* _SIGNAL_MANIPULATION_H_ */

