/* Copyright (C) 2004-2013 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 *          rzander@users.berlios.de
 */

#ifndef _FINITE_ELEMENT_LINEAR_EXTERNAL_LUMPED_NODE_H_
#define _FINITE_ELEMENT_LINEAR_EXTERNAL_LUMPED_NODE_H_

#include <fmatvec/fmatvec.h>
#include "mbsim/mbsim_event.h"
#include "mbsimFlexibleBody/discretization_interface.h"

namespace MBSimFlexibleBody {

  class FiniteElementLinearExternalLumpedNode : public DiscretizationInterface {

    public:
      /*!
       * \brief constructor
       */
      FiniteElementLinearExternalLumpedNode(double& mij_, fmatvec::Vec3& u0_, const fmatvec::Mat3xV& phi_);

      /*!
       * \brief destructor
       */
      virtual ~FiniteElementLinearExternalLumpedNode();

      /* INTERFACE OF DISCRETIZATIONINTERFACE */
      virtual const fmatvec::SymMat& getM() const;
      virtual const fmatvec::Vec& geth() const;
      virtual const fmatvec::SqrMat& getdhdq() const;
      virtual const fmatvec::SqrMat& getdhdu() const;
      virtual int getqSize() const;
      virtual int getuSize() const;
      virtual void computeM(const fmatvec::Vec& qG);
      virtual void computeh(const fmatvec::Vec& qG, const fmatvec::Vec& qGt);
      virtual void computedhdz(const fmatvec::Vec& qG, const fmatvec::Vec& qGt);
      virtual double computeKineticEnergy(const fmatvec::Vec& qG, const fmatvec::Vec& qGt);
      virtual double computeGravitationalEnergy(const fmatvec::Vec& qG);
      virtual double computeElasticEnergy(const fmatvec::Vec& qG);
      virtual fmatvec::Vec3 getPosition(const fmatvec::Vec& qElement, double s);
      virtual fmatvec::SqrMat3 getOrientation(const fmatvec::Vec& qElement, double s);
      virtual fmatvec::Vec3 getVelocity (const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, double s);
      virtual fmatvec::Vec3 getAngularVelocity(const fmatvec::Vec& qElement, const fmatvec::Vec& qpElement, double s);
      virtual fmatvec::Mat getJacobianOfMotion(const fmatvec::Vec& qElement, double s);
      /***************************************************/

      const double getMij() const { return mij; }
      const fmatvec::Vec3& getU0() const { return u0; }
      const fmatvec::Mat3xV& getModeShape() const { return phi; }

    private:
      /*!
       * \brief lumped mass
       */
      const double mij;
      /*!
       * \brief undeformed position vector
       */
      const fmatvec::Vec3 u0;
      /*!
       * \brief mode shape vector: 3*nf
       */
      fmatvec::Mat3xV phi;
  };

}

#endif
