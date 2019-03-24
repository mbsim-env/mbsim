/* Copyright (C) 2004-2019 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _GRAVITY_FUNCTION_H_
#define _GRAVITY_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSimPhysics {

  /**
   * \brief function describing the gravitional acceleration of earth
   * \author Martin Foerg
   */
  class GravityFunction : public MBSim::Function<double(double)> {
    public:
      /** 
       * \brief constructor
       */
      GravityFunction() = default;

      double operator()(const double& h) override { return g0*pow(r/(r+h),2); }
      void initializeUsingXML(xercesc::DOMElement *element) override;

      void setStandardGravity(double g0_) { g0=g0_; }
      void setMeanRadius(double r_) { r=r_; }

    protected:
      double g0{9.80665};
      double r{6371e3};
  };

}

#endif
