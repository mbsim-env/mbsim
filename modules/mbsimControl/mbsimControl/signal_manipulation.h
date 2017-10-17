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

#ifndef _SIGNAL_MANIPULATION_H_
#define _SIGNAL_MANIPULATION_H_

#include "mbsimControl/signal_.h"
#include <mbsim/functions/function.h>

namespace MBSim {
  typedef int Index;
}

namespace MBSimControl {

  /*!
   * \brief Multiplexer
   * \author Markus Schneider
   */
  class Multiplexer : public Signal {
    public:
      Multiplexer(const std::string &name="") : Signal(name) { }
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage, const MBSim::InitConfigSet &config);
      void addInputSignal(Signal * signal_) { signal.push_back(signal_); }
      void updateSignal();
      int getSignalSize() const;
    private:
      std::vector<Signal*> signal;
      std::vector<std::string> signalString;
  };

  /*!
   * \brief Demultiplexer
   * \author Markus Schneider
   */
  class Demultiplexer : public Signal {
    public:
      Demultiplexer(const std::string &name="") : Signal(name), signal(NULL) { }
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage, const MBSim::InitConfigSet &config);
      void setInputSignal(Signal *signal_) { signal = signal_; }
      void setIndices(const std::vector<MBSim::Index> &index_) { index = index_; }
      void updateSignal();
      int getSignalSize() const { return index.size(); }
    private:
      Signal *signal;
      std::vector<MBSim::Index> index;
      std::string signalString;
  };

  /*!
   * \brief SignalTimeDiscretization
   * \author Markus Schneider
   */
  class SignalTimeDiscretization : public Signal {  
    public:
      SignalTimeDiscretization(const std::string &name="") : Signal(name), s(NULL), tOld(-99e99) { }
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage, const MBSim::InitConfigSet &config);
      void setInputSignal(Signal *signal_) { s=signal_; }
      void updateSignal();
      int getSignalSize() const { return s->getSignalSize(); }
    private:
      Signal *s;
      double tOld;
      std::string signalString;
  };

  /*!
   * \brief SignalOperation
   * \author Martin Foerg
   */
  class SignalOperation : public Signal {
    public:
      SignalOperation(const std::string &name="") : Signal(name), f1(NULL), f2(NULL) { }
      ~SignalOperation() { delete f1; delete f2; }
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage, const MBSim::InitConfigSet &config);
      void setInputSignal(Signal *signal_) { signal.resize(1,signal_); }
      void addInputSignal(Signal *signal_) { signal.push_back(signal_); }
      void setFunction(MBSim::Function<fmatvec::VecV(fmatvec::VecV)> *f) {
        f1=f;
        f1->setParent(this);
        f1->setName("Function");
      };
      void setFunction(MBSim::Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *f) {
        f2=f;
        f2->setParent(this);
        f2->setName("Function");
      };
      void updateSignal() { (this->*updateSignal_)(); }
      void updateSignal1();
      void updateSignal2();
      int getSignalSize() const { return f1?f1->getRetSize().first:f2->getRetSize().first; }
    private:
      std::vector<Signal*> signal;
      std::vector<std::string> signalString;
      MBSim::Function<fmatvec::VecV(fmatvec::VecV)> *f1;
      MBSim::Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *f2;
      void (SignalOperation::*updateSignal_)();
  };

}

#endif /* _SIGNAL_MANIPULATION_H_ */
