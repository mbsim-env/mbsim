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

#ifndef _FLEXIBILITY_INFLUENCE_FUNCTION_H_
#define _FLEXIBILITY_INFLUENCE_FUNCTION_H_

#include "mbsim/functions/kinetics/influence_function.h"
#include "mbsim/utils/eps.h"

namespace MBSim {

  /*
   * \brief Influence function for flexibility of contour with no influence to other contours (or contour points)
   */
  class FlexibilityInfluenceFunction : public InfluenceFunction {
    public:
      FlexibilityInfluenceFunction() : flexibility(0) {
      }
      FlexibilityInfluenceFunction(const std::string& ContourName_, const double & flexibility_) :
          flexibility(flexibility_) {
      }
      virtual ~FlexibilityInfluenceFunction() {}
      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const std::pair<Contour*, ContourFrame*>& firstContourInfo, const std::pair<Contour*, ContourFrame*>& secondContourInfo) {
        if(nrm2(getZeta(firstContourInfo)- getZeta(secondContourInfo)) < macheps())
          return flexibility;
        else
          return 0;
      }
      virtual void initializeUsingXML(xercesc::DOMElement *element);
      /***************************************************/

    protected:
      double flexibility;
  };

}

#endif
