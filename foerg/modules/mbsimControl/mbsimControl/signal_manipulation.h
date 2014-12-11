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

namespace MBSimControl {

  /*!
   * \brief SignalAddition
   * \author Markus Schneider
   */
  class SignalAddition : public Signal {
    public:
      SignalAddition(const std::string &name="") : Signal(name) {}
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage);
      void addSignal(Signal * signal, double factor=1.);
      void updateStateDependentVariables(double t);
      int getSignalSize() { return signals[0]->getSignalSize(); }
    private:
      std::vector<Signal *> signals;
      std::vector<double> factors;
      std::vector<std::string> signalString;
      std::vector<double> factorsTmp;
  };

  /*!
   * \brief SignalOffset
   * \author Markus Schneider
   */
  class SignalOffset : public Signal {
    public:
      SignalOffset(const std::string &name="") : Signal(name), signal(0), offset(0, fmatvec::NONINIT), signalString("") {}
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage);
      void setSignal(Signal * s) {signal=s; }
      void setOffset(fmatvec::VecV o) {offset=o; }
      void updateStateDependentVariables(double t);
    private:
      Signal * signal;
      fmatvec::VecV offset;
      std::string signalString;
  };

  /*!
   * \brief SignalMultiplication
   * \author Markus Schneider
   */
  class SignalMultiplication : public Signal {
    public:
      SignalMultiplication(const std::string &name="") : Signal(name) {}
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage);
      void addSignal(Signal * signal, double exp);
      void updateStateDependentVariables(double t);
    private:
      std::vector<Signal *> signals;
      std::vector<double> exponents;
      std::vector<std::string> signalString;
      std::vector<double> exponentsTmp;
  };


  /*!
   * \brief SignalMux
   * \author Markus Schneider
   */
  class SignalMux : public Signal {  
    public:
      SignalMux(const std::string &name="") : Signal(name) {}
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage);
      void addSignal(Signal * signal) {signals.push_back(signal); }
      void updateStateDependentVariables(double t);
    private:
      std::vector<Signal *> signals;
      std::vector<std::string> signalString;
  };


  /*!
   * \brief SignalDemux
   * \author Markus Schneider
   */
  class SignalDemux : public Signal {  
    public:
      SignalDemux(const std::string &name="") : Signal(name), totalSignalSize(0) {}
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage);
      void addSignal(Signal * signal, fmatvec::VecInt index) {signals.push_back(signal); indizes.push_back(index); }
      void updateStateDependentVariables(double t);
    private:
      std::vector<Signal *> signals;
      std::vector<fmatvec::VecInt > indizes;
      std::vector<fmatvec::Vec> indizesTmp;
      std::vector<std::string> signalString;
      int totalSignalSize;
  };


  /*!
   * \brief SignalLimitation
   * \author Markus Schneider
   */
  class SignalLimitation : public Signal {  
    public:
      SignalLimitation(const std::string &name="") : Signal(name), s(NULL), minValue(), maxValue(), signalString("") {}
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage);
      void setMinimalValue(fmatvec::VecV minValue_) {minValue=minValue_; }
      void setMaximalValue(fmatvec::VecV maxValue_) {maxValue=maxValue_; }
      void setSignal(Signal * signal_) {s=signal_; }
      void updateStateDependentVariables(double t);
    private:
      Signal * s;
      fmatvec::VecV minValue, maxValue;
      std::string signalString;
  };


  /*!
   * \brief SignalTimeDiscretization
   * \author Markus Schneider
   */
  class SignalTimeDiscretization : public Signal {  
    public:
      SignalTimeDiscretization(const std::string &name="") : Signal(name), s(NULL), tOld(-99e99), signalString("") {}
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage);
      void setSignal(Signal * signal_) {s=signal_; }
      void updateg(double t);
      void updateStateDependentVariables(double t);
      int getSignalSize() { return s->getSignalSize(); }
    private:
      Signal * s;
      double tOld;
      std::string signalString;
  };


  /*!
   * \brief SignalOperation according <cmath>
   * \author Markus Schneider
   */
  class SignalOperation : public Signal {  
    public:
      SignalOperation(const std::string &name="") : Signal(name), s(NULL), s2(NULL), signalString(""), signal2String(""), op(0), s2values(0, fmatvec::NONINIT) {}
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage);
      void setSignal(Signal * signal_) {s=signal_; }
      void setSecondSignal(Signal * signal_) {s2=signal_; }
      void setSecondSignalValues(fmatvec::VecV s2_) {s2values=s2_; }
      void setOperator(unsigned int op_) {op=op_; };
      void updateStateDependentVariables(double t);
    private:
      Signal * s;
      Signal * s2;
      std::string signalString;
      std::string signal2String;
      unsigned int op;
      fmatvec::VecV s2values;
  };


  /*!
   * \brief SpecialSignalOperation with advanced functionality
   * \author Markus Schneider
   */
  class SpecialSignalOperation : public Signal {  
    public:
      SpecialSignalOperation(const std::string &name="") : Signal(name), s(NULL), s2(NULL), signalString(""), signal2String(""), op(0), s2values(0, fmatvec::NONINIT) {}
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage);
      void setSignal(Signal * signal_) {s=signal_; }
      void setSecondSignal(Signal * signal_) {s2=signal_; }
      void setSecondSignalValues(fmatvec::VecV s2_) {s2values=s2_; }
      void setOperator(unsigned int op_) {op=op_; };
      void updateStateDependentVariables(double t);
    private:
      Signal * s;
      Signal * s2;
      std::string signalString;
      std::string signal2String;
      unsigned int op;
      fmatvec::VecV s2values;
  };

  /*!
   * \brief PID controller
   * \author Martin Foerg
   */
  class PIDController : public Signal {

    public:   
      PIDController(const std::string& name="") : Signal(name), s(NULL), sd(NULL) {}
      void initializeUsingXML(xercesc::DOMElement * element);
      
      void calcxSize() {xSize=updateSignalMethod==&PIDController::updateSignalPD?0:1;}
      
      void init(InitStage stage);

      void updatedx(double t, double dt);
      void updatexd(double t);
      
      void plot(double t,double dt);
     
      void setPID(double P_, double I_, double D_);
      void setInputSignal(Signal *inputSignal_) {s=inputSignal_; }
      void setDerivativeOfInputSignal(Signal *inputSignal_) {sd=inputSignal_; }

      void updateStateDependentVariables(double t);

    protected:
      double P,I,D;
      Signal *s, *sd;
      std::string sString, sdString;
      void (PIDController::*updateSignalMethod)();
      void updateSignalPD();
      void updateSignalPID();
  };

  /*!
   * \brief UnarySignalOperation
   * \author Martin Foerg
   */
  class UnarySignalOperation : public Signal {  
    public:
      UnarySignalOperation(const std::string &name="") : Signal(name), s(NULL), signalString(""), f(0) {}
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage);
      void setSignal(Signal *signal_) {s=signal_; }
      void setFunction(MBSim::Function<fmatvec::VecV(fmatvec::VecV)> *f_) {
        f=f_;
        f->setParent(this);
        f->setName("Function");
      };
      void updateStateDependentVariables(double t);
    private:
      Signal *s;
      std::string signalString;
      MBSim::Function<fmatvec::VecV(fmatvec::VecV)> *f;
  };

  /*!
   * \brief BinarySignalOperation
   * \author Martin Foerg
   */
  class BinarySignalOperation : public Signal {  
    public:
      BinarySignalOperation(const std::string &name="") : Signal(name), s1(NULL), s2(NULL), signal1String(""), signal2String(""), f(0) {}
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage);
      void setSignal1(Signal *signal_) {s1=signal_; }
      void setSignal2(Signal *signal_) {s2=signal_; }
      void setFunction(MBSim::Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *f_) {
        f=f_;
        f->setParent(this);
        f->setName("Function");
      };
      void updateStateDependentVariables(double t);
    private:
      Signal *s1, *s2;
      std::string signal1String, signal2String;
      MBSim::Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *f;
  };

}

#endif /* _SIGNAL_MANIPULATION_H_ */

