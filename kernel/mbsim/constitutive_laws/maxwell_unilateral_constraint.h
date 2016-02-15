/* Copyright (C) 2004-2014 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _MAXWELL_UNILATERAL_CONSTRAINT_H_
#define _MAXWELL_UNILATERAL_CONSTRAINT_H_

#include <mbsim/constitutive_laws/generalized_force_law.h>

namespace MBSim {

  /*!
   * \brief A force law that computes the normal force of many contact kinematics based on the Maxwell-Force-Law
   * \author Kilian Grundl
   * \date 30-07-2012 start of development
   */
  class MaxwellUnilateralConstraint : public GeneralizedForceLaw {
    public:
      /*!
       * \brief constructor
       */
      MaxwellUnilateralConstraint() { }

      /*!
       * \brief destructor
       */
      virtual ~MaxwellUnilateralConstraint() { }

      /* INHERITED INTERFACE */
      virtual bool isClosed(double g, double gTol) { return g < gTol ? true : false; }
      virtual bool remainsClosed(double s, double sTol) {return true; }
      virtual bool isSetValued() const { return false; }
  };

}

#endif
