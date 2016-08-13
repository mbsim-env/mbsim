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

#ifndef _INFLUENCE_FUNCTION_H_
#define _INFLUENCE_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {
  class ContourFrame;
  class Contour;
}

namespace fmatvec {

  //! We need to define fmatvec::StaticSize for the special arguement types defined by InfluenceFunction.
  template<>
  struct StaticSize<std::pair<MBSim::Contour*, MBSim::ContourFrame*>> {
    enum { size1=-1, size2=-1 }; // define as -1 = not meaningful
  };

}

namespace MBSim {

  /**
   * \brief function describing the influence between the deformations on a body
   */
  class InfluenceFunction : public Function<double(std::pair<Contour*, ContourFrame*>, std::pair<Contour*, ContourFrame*>)> {
    public:
      InfluenceFunction(){}
      virtual ~InfluenceFunction() {}
      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const std::pair<Contour*, ContourFrame*>& firstContourInfo, const std::pair<Contour*, ContourFrame*>& secondContourInfo)=0;
      virtual void initializeUsingXML(xercesc::DOMElement *element);
      /***************************************************/
    protected:
      fmatvec::Vec2 evalZeta(const std::pair<Contour*, ContourFrame*>& contourInfo);
  };

}

#endif
