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

  /**
   * \brief function describing the influence between the deformations on a body
   */
  class InfluenceFunction : public Function<double(std::pair<Contour*, ContourFrame*>, std::pair<Contour*, ContourFrame*>)> {
    public:
      InfluenceFunction(){}
      virtual ~InfluenceFunction() {}
      /* INHERITED INTERFACE OF FUNCTION2 */
      void setTime(double t_) { t = t_; }
      virtual double operator()(const std::pair<Contour*, ContourFrame*>& firstContourInfo, const std::pair<Contour*, ContourFrame*>& secondContourInfo)=0;
      virtual void initializeUsingXML(xercesc::DOMElement *element);
      /***************************************************/
    protected:
      double t;
      fmatvec::Vec2 getZeta(double t, const std::pair<Contour*, ContourFrame*>& contourInfo);
  };

}

#endif
