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

#ifndef _UNILATERAL_CONSTRAINT_H_
#define _UNILATERAL_CONSTRAINT_H_

#include <mbsim/constitutive_laws/generalized_force_law.h>

namespace MBSim {

  /**
   * \brief basic unilateral force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   */
  class UnilateralConstraint : public GeneralizedForceLaw {
    public:
      /**
       * \brief constructor
       */
      UnilateralConstraint() = default;

      /**
       * \brief destructor
       */
      ~UnilateralConstraint() override = default;

      /* INHERITED INTERFACE */
      bool isClosed(double g, double gTol) override { return g<=gTol; }
      bool remainsClosed(double gd, double gdTol) override { return gd<=gdTol; }
      double project(double la, double gdn, double r, double laMin=0) override;
      fmatvec::Vec diff(double la, double gdn, double r, double laMin=0) override;
      double solve(double G, double gdn) override;
      bool isFulfilled(double la,  double gdn, double tolla, double tolgd, double laMin=0) override;
      bool isSetValued() const override { return true; }
      /***************************************************/
  };

}

#endif
