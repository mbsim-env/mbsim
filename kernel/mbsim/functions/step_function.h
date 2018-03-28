/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _STEP_FUNCTION_H_
#define _STEP_FUNCTION_H_

#include "mbsim/functions/function.h"
#include "mbsim/utils/utils.h"

namespace MBSim {

  template<typename Sig> class StepFunction; 

  template<typename Ret, typename Arg>
  class StepFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    private:
      double stepTime, stepSize, stepInit;
    public:
      StepFunction() = default;
      StepFunction(double stepTime_, double stepSize_, double stepInit_=0.) : stepTime(stepTime_), stepSize(stepSize_), stepInit(stepInit_) { }
      int getArgSize() const override { return 1; }
      std::pair<int, int> getRetSize() const override { return std::make_pair(1,1); }
      Ret operator()(const Arg &x) override {
        return FromDouble<Ret>::cast((ToDouble<Arg>::cast(x)>=stepTime)?stepSize:stepInit);
      }
      void initializeUsingXML(xercesc::DOMElement *element) override {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"time");
        stepTime=MBXMLUtils::E(e)->getText<double>();
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"size");
        stepSize=MBXMLUtils::E(e)->getText<double>();
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"init");
        stepSize=MBXMLUtils::E(e)->getText<double>();
      }
  };

}

#endif
