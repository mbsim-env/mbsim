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

#ifndef _GENERALIZED_FORCE_LAW_H_
#define _GENERALIZED_FORCE_LAW_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  /**
   * \brief basic force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   */
  class GeneralizedForceLaw : public Element {
    public:
      /**
       * \brief constructor
       */
      GeneralizedForceLaw() : Element(uniqueDummyName(this)) {
        plotFeature[plotRecursive]=false;
      }

      /**
       * \brief destructor
       */
      ~GeneralizedForceLaw() override = default;

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \brief decides, if force law is active
       * \param gap distance
       * \param tolerance
       * \return flag, if force law is active
       */
      virtual bool isClosed(double g, double gTol) { return true; }

      /**
       * \brief prox function evaluation
       * \param kinetic variable
       * \param kinematic variable
       * \param relaxation factor
       * \param minimal threshold for kinetic variable
       * \return result of prox function evaluation
       */
      virtual double project(double la, double gdn, double r, double laMin=0) { throwError("(GeneralizedForceLaw::project): Not implemented."); }
      virtual fmatvec::Vec diff(double la, double gdn, double r, double laMin=0) { throwError("(GeneralizedForceLaw::diff): Not implemented."); }
      virtual double solve(double G, double gdn) { throwError("(GeneralizedForceLaw::solve): Not implemented."); }
      virtual double operator()(double g, double gd) { throwError("(GeneralizedForceLaw::operator()): Not implemented."); }

      /**
       * \param contact force parameter
       * \param contact relative velocity
       * \param tolerance for contact force parameters
       * \param tolerance for relative velocity
       * \return flag if the force law is valid given the parameters
       */
      virtual bool isFulfilled(double la,  double gdn, double tolla, double tolgd, double laMin=0) { return true; }

      /**
       * \return flag if the force law is setvalued
       */
      virtual bool isSetValued() const = 0;
  };

}

#endif
