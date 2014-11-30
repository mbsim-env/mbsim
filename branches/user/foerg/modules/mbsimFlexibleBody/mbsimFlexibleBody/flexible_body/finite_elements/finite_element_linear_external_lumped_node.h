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
#include <mbsim/discretization_interface.h>

namespace MBSimFlexibleBody {

  class FiniteElementLinearExternalLumpedNode : public MBSim::DiscretizationInterface {

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
      virtual fmatvec::Vec computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& qG, const MBSim::ContourPointData& data);
      /***************************************************/

      const double getMij() const;
//      fmatvec::Vec getU();
      const fmatvec::Vec3& getU0() const;
      const fmatvec::Mat3xV& getModeShape() const;

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

  inline const fmatvec::SymMat& FiniteElementLinearExternalLumpedNode::getM() const { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::getM): Not implemented");  }
  inline const fmatvec::Vec& FiniteElementLinearExternalLumpedNode::geth() const { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::geth): Not implemented");}
  inline const fmatvec::SqrMat& FiniteElementLinearExternalLumpedNode::getdhdq() const { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::getdhdq): Not implemented");}
  inline const fmatvec::SqrMat& FiniteElementLinearExternalLumpedNode::getdhdu() const {throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::getdhdu): Not implemented");}
  inline int FiniteElementLinearExternalLumpedNode::getqSize() const { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::getqSize): Not implemented"); }
  inline int FiniteElementLinearExternalLumpedNode::getuSize() const { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::getuSize): Not implemented");}
  inline void  FiniteElementLinearExternalLumpedNode::computeM(const fmatvec::Vec& qG) { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::computeM): Not implemented"); }
  inline void  FiniteElementLinearExternalLumpedNode::computeh(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::computeh): Not implemented"); }
  inline void  FiniteElementLinearExternalLumpedNode::computedhdz(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::computedhdz): Not implemented"); }
  inline double FiniteElementLinearExternalLumpedNode::computeKineticEnergy(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::computeKineticEnergy): Not implemented"); }
  inline double FiniteElementLinearExternalLumpedNode::computeGravitationalEnergy(const fmatvec::Vec& qG) { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::computeGravitationalEnergy): Not implemented"); }
  inline double FiniteElementLinearExternalLumpedNode::computeElasticEnergy(const fmatvec::Vec& qG) { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::computeElasticEnergy): Not implemented"); }
  inline fmatvec::Vec FiniteElementLinearExternalLumpedNode::computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::computePosition): Not implemented!"); }
  inline fmatvec::SqrMat FiniteElementLinearExternalLumpedNode::computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::computeOrientation): Not implemented!"); }
  inline fmatvec::Vec FiniteElementLinearExternalLumpedNode::computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::computeVelocity): Not implemented!"); }
  inline fmatvec::Vec FiniteElementLinearExternalLumpedNode::computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::computeAngularVelocity): Not implemented!"); }
  inline fmatvec::Mat FiniteElementLinearExternalLumpedNode::computeJacobianOfMotion(const fmatvec::Vec& qG,const MBSim::ContourPointData& data) { throw MBSim::MBSimError("(FiniteElementLinearExternalLumpedNode::computeJacobianOfMotion): Not implemented"); }


  inline const double FiniteElementLinearExternalLumpedNode::getMij() const {return mij;}
  inline const fmatvec::Vec3& FiniteElementLinearExternalLumpedNode::getU0() const {return u0;}
  inline const fmatvec::Mat3xV& FiniteElementLinearExternalLumpedNode::getModeShape() const {return phi;}

}

#endif
