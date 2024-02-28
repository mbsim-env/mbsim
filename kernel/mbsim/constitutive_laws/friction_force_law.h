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

#ifndef _FRICTION_FORCE_LAW_H_
#define _FRICTION_FORCE_LAW_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  class FrictionImpactLaw;

  /**
   * \brief basic friction force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   */
  class FrictionForceLaw : public Element {
    public:
      /**
       * \brief constructor
       */
      FrictionForceLaw() : Element(uniqueDummyName(this)) {
        plotFeature[plotRecursive]=false;
      }

      /**
       * \brief destructor
       */
      ~FrictionForceLaw() override = default;

      /* INTERFACE FOR DERIVED CLASSES */
      virtual fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r) { throwError("(FrictionForceLaw::project): Not implemented."); }
      virtual fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r) { throwError("(FrictionForceLaw::diff): Not implemented."); }
      virtual fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, double laN) { throwError("(FrictionForceLaw::solve): Not implemented."); }
      virtual bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double tolla, double tolgd) { return true; }
      virtual fmatvec::Vec operator()(const fmatvec::Vec &gd, double laN) { throwError("(FrictionForceLaw::operator()): Not implemented."); }
      virtual fmatvec::Vec dlaTdlaN(const fmatvec::Vec& gd) { throwError("(FrictionForceLaw::dlaTdlaN): Not implemented."); }
      virtual int getFrictionDirections() = 0;
      virtual bool isSticking(const fmatvec::Vec& s, double sTol) = 0;
      virtual bool isSetValued() const = 0;
      virtual FrictionImpactLaw* createFrictionImpactLaw() const { return nullptr; }
      /***************************************************/
  };

}

#endif
