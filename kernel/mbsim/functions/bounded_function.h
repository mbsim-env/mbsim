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

#ifndef _BOUNDED_FUNCTION_H_
#define _BOUNDED_FUNCTION_H_

#include "mbsim/functions/function.h"
#include <limits>

namespace MBSim {

  template<typename Sig> class BoundedFunction; 

  template<typename Ret, typename Arg>
  class BoundedFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    private:
      double lowerBound, upperBound;
    public:
      BoundedFunction() : lowerBound(-std::numeric_limits<double>::max()), upperBound(std::numeric_limits<double>::max()) { }
      void setLowerBound(double lowerBound_) { lowerBound = lowerBound_; }
      void setUpperBound(double upperBound_) { upperBound = upperBound_; }
      int getArgSize() const { return 1; }
      std::pair<int, int> getRetSize() const { return std::make_pair(1,1); }
      Ret operator()(const Arg &x_) {
        double x = ToDouble<Arg>::cast(x_);
        if(x<lowerBound)
          x = lowerBound;
        if(x>upperBound)
          x = upperBound;
        return FromDouble<Ret>::cast(x);
      }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e;
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"lowerBound");
        lowerBound=MBXMLUtils::E(e)->getText<double>();
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"upperBound");
        upperBound=MBXMLUtils::E(e)->getText<double>();
      }
  };

}

#endif
