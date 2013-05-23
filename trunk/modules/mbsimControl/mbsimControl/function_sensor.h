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

#ifndef _FUNCTION_SENSOR_H_
#define _FUNCTION_SENSOR_H_

#include "mbsimControl/sensor.h"
#include "mbsim/utils/function.h"

namespace MBSimControl {

  /*!
   * \brief FunctionSensor
   * \author Markus Schneider
   */
  class FunctionSensor : public Sensor {
    public:
      FunctionSensor(const std::string &name="") : Sensor(name), function(NULL), y() {}
      FunctionSensor(const std::string &name, MBSim::Function1<fmatvec::Vec, double>* function_);
      std::string getType() const { return "FunctionSensor"; }
      void setFunction(MBSim::Function1<fmatvec::Vec, double>* function_);
      fmatvec::Vec getSignal() {return y.copy(); }
      void updateg(double t);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      MBSim::Function1<fmatvec::Vec, double> * function;
      fmatvec::Vec y;
  };

  /*!
   * \brief Function1_SSEvaluation
   * \author Markus Schneider
   */
  class Function1_SSEvaluation : public Signal {
    public:
      Function1_SSEvaluation(const std::string &name="") : Signal(name), signal(NULL), fun(NULL), signalString("") {}
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void setSignal(Signal * s) {signal=s; }
      void setFunction(MBSim::Function1<double, double>* fun_) {fun=fun_; }
      fmatvec::Vec getSignal();
    private:
      Signal * signal;
      MBSim::Function1<double, double>* fun;
      std::string signalString;
  };

  /*!
   * \brief Function1_SSSEvaluation
   * \author Markus Schneider
   */
  class Function2_SSSEvaluation : public Signal {
    public:
      Function2_SSSEvaluation(const std::string &name="") : Signal(name), signal1(NULL), signal2(NULL), fun(NULL), signal1String(""), signal2String("") {}
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void setSignals(Signal * s1, Signal * s2) {signal1=s1; signal2=s2; }
      void setFunction(MBSim::Function2<double, double, double>* fun_) {fun=fun_; }
      fmatvec::Vec getSignal();
    private:
      Signal * signal1;
      Signal * signal2;
      MBSim::Function2<double, double, double>* fun;
      std::string signal1String;
      std::string signal2String;
  };

}

#endif /* _FUNCTION_SENSOR_H_ */

