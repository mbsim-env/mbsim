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

#ifndef _BILATERAL_CONSTRAINT_H_
#define _BILATERAL_CONSTRAINT_H_

#include <mbsim/constitutive_laws/generalized_force_law.h>

namespace MBSim {

  /**
   * \brief basic bilateral force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \date 2010-07-06 isSticking added for impact laws (Robert Huber)
   */
  class BilateralConstraint : public GeneralizedForceLaw {
    public:
      /**
       * \brief constructor
       */
      BilateralConstraint() { }

      /**
       * \brief destructor
       */
      virtual ~BilateralConstraint() { }

      /* INHERITED INTERFACE */
      virtual bool isClosed(double g, double gTol) { return true; }
      virtual double project(double la, double gdn, double r, double laMin=0);
      virtual fmatvec::Vec diff(double la, double gdn, double r, double laMin=0);
      virtual double solve(double G, double gdn);
      virtual bool isFulfilled(double la,  double gdn, double tolla, double tolgd, double laMin=0);
      virtual bool isSetValued() const { return true; }
      /***************************************************/

      bool remainsClosed(double s, double sTol) { return true; }

      virtual std::string getType() const { return "BilateralConstraint"; }
  };

}

#endif
