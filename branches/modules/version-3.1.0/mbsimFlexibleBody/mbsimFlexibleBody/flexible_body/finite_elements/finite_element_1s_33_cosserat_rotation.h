/* Copyright (C) 2004-2012 MBSim Development Team
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
 */

#ifndef _FINITE_ELEMENT_1S_33_COSSERAT_ROTATION_H_
#define _FINITE_ELEMENT_1S_33_COSSERAT_ROTATION_H_

#include "mbsim/discretization_interface.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/contour_pdata.h"
#include "mbsimFlexibleBody/pointer.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_cosserat_rotation.h"
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsim/utils/eps.h"

#include "fmatvec.h"

namespace MBSimFlexibleBody {

  /**
   * \brief finite element for spatial beam using Cosserat model : rotation element for bending and torsion
   * \author Thorsten Schindler
   * \author Christian Käsbauer
   * \author Thomas Cebulla
   * \date 2011-10-11 initial commit (Thorsten Schindler)
   * \todo bending/torsion in rhs TODO
   *
   * Cosserat model based on
   * H. Lang, J. Linn, M. Arnold: Multi-body dynamics simulation of geometrically exact Cosserat rods
   * but with 
   *  - Kirchhoff assumption (-> less stiff)
   *  - Cardan parametrisation (-> less problems with condition and drift for quaternion dae system)
   *  - piecewise constant Darboux vector with evaluation according to
   *    I. Romero: The interpolation of rotations and its application to finite element models of
   *    geometrically exact beams
   */
  class FiniteElement1s33CosseratRotation : public MBSim::DiscretizationInterface<fmatvec::Ref> {
    public:

      /**
       * \brief constructor
       * \param length of finite element
       * \param Young's modulus
       * \param shear modulus
       * \param first area moment of inertia
       * \param second area moment of inertia
       * \param torsional moment of inertia
       * \param cardan object
       */
      FiniteElement1s33CosseratRotation(double l0_,double E_,double G_,double I1_,double I2_,double I0_,CardanPtr ag_);

      /**
       * \brief destructor
       */
      virtual ~FiniteElement1s33CosseratRotation();

      /* INHERITED INTERFACE OF DISCRETIZATIONINTERFACE */ 
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

      virtual fmatvec::Vec3 computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::SqrMat3 computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec3 computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec3 computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& qG, const MBSim::ContourPointData& data);

      /* GETTER / SETTER */
      void setCurlRadius(double R1,double R2);		
      double getl0() const;

      /**
       * \brief compute JACOBIAN of contact description in global coordinates
       * \param global coordinates
       * \param LAGRANGIAN parameter
       */
      fmatvec::Mat computeJXqG(const fmatvec::Vec& qG,double x);

    private:
      /**
       * \brief length of finite element
       */
      double l0;

      /**
       * \brief Young's modulus 
       */
      double E;

      /**
       * \brief shear modulus 
       */
      double G;

      /**
       * \brief geometrical moment of inertia 
       */
      double I1, I2, I0;

      /**
       * \brief predefined bending
       * k10: precurvature in t-b-plane
       * k20: precurvature in t-n-plane
       */
      double k10, k20;

      /**
       * \brief global system description 
       */
      fmatvec::Vec h;

      /**
       * \brief state at Lagrangian coordinate
       */
      fmatvec::Vec X;

      /**
       * \brief matrices for implicit integration 
       */
      fmatvec::SqrMat dhdq, dhdu;

      /**
       * \brief Cardan-object 
       */
      CardanPtr ag;

      FiniteElement1s33CosseratRotation(); // standard constructor
      FiniteElement1s33CosseratRotation(const FiniteElement1s33CosseratRotation&); // copy constructor
      FiniteElement1s33CosseratRotation& operator=(const FiniteElement1s33CosseratRotation&); // assignment operator
  };

  inline const fmatvec::SymMat& FiniteElement1s33CosseratRotation::getM() const { throw MBSim::MBSimError("Error(FiniteElement1s33CosseratRotation::getM): Not implemented");  }
  inline const fmatvec::Vec& FiniteElement1s33CosseratRotation::geth() const { return h; }
  inline const fmatvec::SqrMat& FiniteElement1s33CosseratRotation::getdhdq() const { return dhdq; }
  inline const fmatvec::SqrMat& FiniteElement1s33CosseratRotation::getdhdu() const { return dhdu; }
  inline int FiniteElement1s33CosseratRotation::getqSize() const { return 9; }
  inline int FiniteElement1s33CosseratRotation::getuSize() const { return 9; }
  inline void  FiniteElement1s33CosseratRotation::computeM(const fmatvec::Vec& qG) { throw MBSim::MBSimError("Error(FiniteElement1s33CosseratRotation::computeM): Not implemented"); }
  inline void  FiniteElement1s33CosseratRotation::computedhdz(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) { throw MBSim::MBSimError("Error(FiniteElement1s33CosseratRotation::computedhdz): Not implemented"); }
  inline double FiniteElement1s33CosseratRotation::computeKineticEnergy(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) { throw MBSim::MBSimError("Error(FiniteElement1s33CosseratRotation::computeKineticEnergy): Not implemented"); }
  inline double FiniteElement1s33CosseratRotation::computeGravitationalEnergy(const fmatvec::Vec& qG) { throw MBSim::MBSimError("Error(FiniteElement1s33CosseratRotation::computeGravitationalEnergy): Not implemented"); }
  inline fmatvec::Vec3 FiniteElement1s33CosseratRotation::computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s33CosseratRotation::computePosition): Not implemented!"); }
  inline fmatvec::SqrMat3 FiniteElement1s33CosseratRotation::computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s33CosseratRotation::computeOrientation): Not implemented!"); }
  inline fmatvec::Vec3 FiniteElement1s33CosseratRotation::computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s33CosseratRotation::computeVelocity): Not implemented!"); }
  inline fmatvec::Vec3 FiniteElement1s33CosseratRotation::computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s33CosseratRotation::computeAngularVelocity): Not implemented!"); }
  inline fmatvec::Mat FiniteElement1s33CosseratRotation::computeJacobianOfMotion(const fmatvec::Vec& qG,const MBSim::ContourPointData& data) { return computeJXqG(qG,data.getLagrangeParameterPosition()(0)); }
  inline double FiniteElement1s33CosseratRotation::getl0() const { return l0; }
  inline fmatvec::Mat FiniteElement1s33CosseratRotation::computeJXqG(const fmatvec::Vec& qG,double x) { throw MBSim::MBSimError("ERROR (FiniteElement1s33CosseratRotation::computeJXqG): Not implemented!"); }

  inline FiniteElement1s33CosseratRotation::FiniteElement1s33CosseratRotation(double l0_, double E_, double G_, double I1_, double I2_, double I0_, CardanPtr ag_) :
      l0(l0_), E(E_), G(G_), I1(I1_), I2(I2_), I0(I0_), k10(0.), k20(0.), h(9, fmatvec::INIT, 0.), X(12, fmatvec::INIT, 0.), ag(ag_) {
  }

  inline FiniteElement1s33CosseratRotation::~FiniteElement1s33CosseratRotation() {
  }

  inline void FiniteElement1s33CosseratRotation::setCurlRadius(double R1, double R2) {
    if (fabs(R1) > MBSim::epsroot())
      k10 = 1. / R1;
    if (fabs(R2) > MBSim::epsroot())
      k20 = 1. / R2;
  }

  inline void FiniteElement1s33CosseratRotation::computeh(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) {
    /* angles */
    fmatvec::Vec phi = (qG(0, 2) + qG(6, 8)) / 2.;
    fmatvec::Vec dphids = (qG(6, 8) - qG(0, 2)) / l0;

    fmatvec::Vec tangent = ag->computet(phi);
    fmatvec::Vec normal = ag->computen(phi);
    fmatvec::Vec binormal = ag->computeb(phi);

    fmatvec::SqrMat dtangentdphi = ag->computetq(phi);
    fmatvec::SqrMat dnormaldphi = ag->computenq(phi);
    fmatvec::SqrMat dbinormaldphi = ag->computebq(phi);

    /* differentiation of 'bending and torsion energy' with respect to qG */
    double GI0ktilde0 = G * I0 * binormal.T() * dnormaldphi * dphids;
    fmatvec::Vec ktilde0_0 = 0.5 * dbinormaldphi.T() * dnormaldphi * dphids;
    fmatvec::Vec ktilde0_1 = dnormaldphi.T() * binormal / l0;
    fmatvec::Vec ktilde0_2 = 0.5 * (ag->computenqt(phi, dphids)).T() * binormal;
    fmatvec::Vec dBTtorsiondqG(9, fmatvec::INIT, 0.);
    dBTtorsiondqG(0, 2) = ktilde0_0 - ktilde0_1 + ktilde0_2;
    dBTtorsiondqG(6, 8) = ktilde0_0 + ktilde0_1 + ktilde0_2;
    dBTtorsiondqG *= GI0ktilde0;

    double EI1ktilde1 = E * I1 * (tangent.T() * dbinormaldphi * dphids - k10);
    fmatvec::Vec ktilde1_0 = 0.5 * dtangentdphi.T() * dbinormaldphi * dphids;
    fmatvec::Vec ktilde1_1 = dbinormaldphi.T() * tangent / l0;
    fmatvec::Vec ktilde1_2 = 0.5 * (ag->computebqt(phi, dphids)).T() * tangent;
    fmatvec::Vec dBTbending1dqG(9, fmatvec::INIT, 0.);
    dBTbending1dqG(0, 2) = ktilde1_0 - ktilde1_1 + ktilde1_2;
    dBTbending1dqG(6, 8) = ktilde1_0 + ktilde1_1 + ktilde1_2;
    dBTbending1dqG *= EI1ktilde1;

    double EI2ktilde2 = E * I2 * (normal.T() * dtangentdphi * dphids - k20);
    fmatvec::Vec ktilde2_0 = 0.5 * dnormaldphi.T() * dtangentdphi * dphids;
    fmatvec::Vec ktilde2_1 = dtangentdphi.T() * normal / l0;
    fmatvec::Vec ktilde2_2 = 0.5 * (ag->computetqt(phi, dphids)).T() * normal;
    fmatvec::Vec dBTbending2dqG(9, fmatvec::INIT, 0.);
    dBTbending2dqG(0, 2) = ktilde2_0 - ktilde2_1 + ktilde2_2;
    dBTbending2dqG(6, 8) = ktilde2_0 + ktilde2_1 + ktilde2_2;
    dBTbending2dqG *= EI2ktilde2;

    /* generalized forces */
    h = (dBTtorsiondqG + dBTbending1dqG + dBTbending2dqG) * (-l0);
    //cout << h << endl;
    //throw;
  }

  inline double FiniteElement1s33CosseratRotation::computeElasticEnergy(const fmatvec::Vec& qG) {
    fmatvec::Vec phi = (qG(0, 2) + qG(6, 8)) / 2.;
    fmatvec::Vec dphids = (qG(6, 8) - qG(0, 2)) / l0;

    fmatvec::Vec tangent = ag->computet(phi);
    fmatvec::Vec normal = ag->computen(phi);
    fmatvec::Vec binormal = ag->computeb(phi);

    fmatvec::SqrMat dtangentdphi = ag->computetq(phi);
    fmatvec::SqrMat dnormaldphi = ag->computenq(phi);
    fmatvec::SqrMat dbinormaldphi = ag->computebq(phi);

    return 0.5 * l0 * (G * I0 * pow(binormal.T() * dnormaldphi * dphids, 2.) + E * I1 * pow(tangent.T() * dbinormaldphi * dphids - k10, 2.) + E * I2 * pow(normal.T() * dtangentdphi * dphids - k20, 2.));
  }

}

#endif /* _FINITE_ELEMENT_1S_33_COSSERAT_ROTATION_H_ */
