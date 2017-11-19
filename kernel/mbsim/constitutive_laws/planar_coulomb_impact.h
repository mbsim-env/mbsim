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

#ifndef _PLANAR_COULOMB_IMPACT_H_
#define _PLANAR_COULOMB_IMPACT_H_

#include <mbsim/constitutive_laws/friction_impact_law.h>

namespace MBSim {

  /**
   * \brief basic planar friction force law on velocity level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   */
  class PlanarCoulombImpact : public FrictionImpactLaw {
    public:
      /**
       * \brief constructor
       */
      PlanarCoulombImpact(double mu_=0) : mu(mu_) { }

      /**
       * \brief destructor
       */
      ~PlanarCoulombImpact() override = default;

      /* INHERITED INTERFACE */
      fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double r) override;
      fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double r) override;
      fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN) override;
      bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double laTol, double gdTol) override;
      int isSticking(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double laTol, double gdTol) override;
      int getFrictionDirections() override { return 1; }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      /***************************************************/

      void setFrictionCoefficient(double mu_) { mu = mu_; }
      double getFrictionCoefficient(double gd) { return mu; }

    protected:
      double mu;
  };

}

#endif
