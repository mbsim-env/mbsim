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

#ifndef _LINEAR_SPRING_DAMPER_FORCE_H_
#define _LINEAR_SPRING_DAMPER_FORCE_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  /**
   * \brief function describing a linear relationship between the input relative distance / velocity and the output for a spring
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   */
  class LinearSpringDamperForce : public Function<double(double,double)> {
    public:
      /** 
       * \brief constructor
       */
      LinearSpringDamperForce() : l0(0) {}

      /** 
       * \brief constructor
       * \param stiffness
       * \param damping
       */
      LinearSpringDamperForce(double c_, double d_) : c(c_), d(d_), l0(0) {}

      /** 
       * \brief constructor
       * \param stiffness
       * \param damping
       * \param undeformed length
       */
      LinearSpringDamperForce(double c_, double d_, double l0_); 

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const double& g, const double& gd) { return c*(g-l0) + d*gd; }
      void initializeUsingXML(xercesc::DOMElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setStiffnessCoefficient(double c_) { c=c_; }
      void setDampingCoefficient(double d_) { d=d_; }
      /***************************************************/

    protected:
      /**
       * \brief stiffness, damping, undeformed length
       */
      double c, d, l0;
  };

}

#endif
