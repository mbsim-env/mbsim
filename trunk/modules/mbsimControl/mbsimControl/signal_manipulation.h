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

namespace MBSimControl {

  /*!
   * \brief SignalAddition
   * \author Markus Schneider
   */
  class SignalAddition : public Signal {
    public:
      SignalAddition(const std::string &name) : Signal(name) {}
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void addSignal(Signal * signal, double factor=1.);
      fmatvec::Vec getSignal();
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
      SignalOffset(const std::string &name) : Signal(name), signal(0), offset(0, fmatvec::NONINIT), signalString("") {}
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void setSignal(Signal * s) {signal=s; }
      void setOffset(fmatvec::Vec o) {offset=o; }
      fmatvec::Vec getSignal();
    private:
      Signal * signal;
      fmatvec::Vec offset;
      std::string signalString;
  };

  /*!
   * \brief SignalMultiplication
   * \author Markus Schneider
   */
  class SignalMultiplication : public Signal {
    public:
      SignalMultiplication(const std::string &name) : Signal(name) {}
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void addSignal(Signal * signal, double exp);
      fmatvec::Vec getSignal();
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
      SignalMux(const std::string &name) : Signal(name) {}
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void addSignal(Signal * signal) {signals.push_back(signal); }
      fmatvec::Vec getSignal();
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
      SignalDemux(const std::string &name) : Signal(name), totalSignalSize(0) {}
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void addSignal(Signal * signal, fmatvec::Vector<int> index) {signals.push_back(signal); indizes.push_back(index); }
      fmatvec::Vec getSignal();
    private:
      std::vector<Signal *> signals;
      std::vector<fmatvec::Vector<int> > indizes;
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
      SignalLimitation(const std::string &name) : Signal(name), s(NULL), minValue(0), maxValue(0), signalString("") {}
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void setMinimalValue(fmatvec::Vec minValue_) {minValue=minValue_; }
      void setMaximalValue(fmatvec::Vec maxValue_) {maxValue=maxValue_; }
      void setSignal(Signal * signal_) {s=signal_; }
      fmatvec::Vec getSignal();
    private:
      Signal * s;
      fmatvec::Vec minValue, maxValue;
      std::string signalString;
  };


  /*!
   * \brief SignalTimeDiscretization
   * \author Markus Schneider
   */
  class SignalTimeDiscretization : public Signal {  
    public:
      SignalTimeDiscretization(const std::string &name) : Signal(name), s(NULL), y(0), tOld(-99e99), signalString("") {}
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void setSignal(Signal * signal_) {s=signal_; }
      void updateg(double t);
      fmatvec::Vec getSignal();
    private:
      Signal * s;
      fmatvec::Vec y;
      double tOld;
      std::string signalString;
  };


  /*!
   * \brief SignalOperation according <cmath>
   * \author Markus Schneider
   */
  class SignalOperation : public Signal {  
    public:
      SignalOperation(const std::string &name) : Signal(name), s(NULL), s2(NULL), signalString(""), signal2String(""), op(0), s2values(0, fmatvec::NONINIT) {}
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void setSignal(Signal * signal_) {s=signal_; }
      void setSecondSignal(Signal * signal_) {s2=signal_; }
      void setSecondSignalValues(fmatvec::Vec s2_) {s2values=s2_; }
      void setOperator(unsigned int op_) {op=op_; };
      fmatvec::Vec getSignal();
    private:
      Signal * s;
      Signal * s2;
      std::string signalString;
      std::string signal2String;
      unsigned int op;
      fmatvec::Vec s2values;
  };


  /*!
   * \brief SpecialSignalOperation with advanced functionality
   * \author Markus Schneider
   */
  class SpecialSignalOperation : public Signal {  
    public:
      SpecialSignalOperation(const std::string &name) : Signal(name), s(NULL), s2(NULL), signalString(""), signal2String(""), op(0), s2values(0, fmatvec::NONINIT) {}
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void setSignal(Signal * signal_) {s=signal_; }
      void setSecondSignal(Signal * signal_) {s2=signal_; }
      void setSecondSignalValues(fmatvec::Vec s2_) {s2values=s2_; }
      void setOperator(unsigned int op_) {op=op_; };
      fmatvec::Vec getSignal();
    private:
      Signal * s;
      Signal * s2;
      std::string signalString;
      std::string signal2String;
      unsigned int op;
      fmatvec::Vec s2values;
  };

}

#endif /* _SIGNAL_MANIPULATION_H_ */

