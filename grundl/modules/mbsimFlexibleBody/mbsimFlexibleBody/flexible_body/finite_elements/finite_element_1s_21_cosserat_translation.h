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

#ifndef _FINITE_ELEMENT_1S_21_COSSERAT_TRANSLATION_H_
#define _FINITE_ELEMENT_1S_21_COSSERAT_TRANSLATION_H_

#include "mbsim/discretization_interface.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/contour_pdata.h"
#include "mbsimFlexibleBody/pointer.h"
#include "fmatvec.h"

namespace MBSimFlexibleBody {

  /**
   * \brief finite translation element for planar beam using Cosserat model
   * \author Thomas Cebulla
   * \author Thorsten Schindler
   * \author Robert von Zitzewitz
   * \date 2012-12-14 initial commit (Thomas Cebulla)
   * \todo computeState only in FlexibleBody TODO
   * \todo implicit integration TODO
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
  class FiniteElement1s21CosseratTranslation : public MBSim::DiscretizationInterface {
    public:

      /**
       * \brief constructor
       * \param length of finite element
       * \param density
       * \param cross-sectional area
       * \param Young's modulus
       * \param shear modulus
       * \param first area moment of inertia
       * \param second area moment of inertia
       * \param torsional moment of inertia
       * \param acceleration of gravity
       * \param cardan object
       */
      FiniteElement1s21CosseratTranslation(double l0_,double rho_,double A_,double E_,double G_,double I1_,double I2_,double I0_,const fmatvec::Vec& g_,CardanPtr ag_);

      /**
       * \brief destructor
       */
      virtual ~FiniteElement1s21CosseratTranslation();		

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

      virtual fmatvec::Vec computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& qG, const MBSim::ContourPointData& data);

      /* GETTER / SETTER */
      void setMaterialDamping(double cEps0D_,double cEps1D_,double cEps2D_);		
      void setShearCorrectionFactors(double sigma1_, double sigma2_);		
      double getl0() const;

      /**
       * \brief compute state of finite element
       * \param global coordinates
       * \param global velocities
       * \param LAGRANGIAN parameter in [0,l0]
       */
      const fmatvec::Vec& computeStateTranslation(const fmatvec::Vec& qG, const fmatvec::Vec& qGt, double s);

      /**
       * \brief compute JACOBIAN of contact description in global coordinates
       * \param global coordinates
       * \param LAGRANGIAN parameter
       */
      fmatvec::Mat computeJXqG(const fmatvec::Vec& qG,double x);

      /**
       * \brief initialize translational part of mass matrix
       */
      void initM();

    private:
      /**
       * \brief length of finite element
       */
      double l0;

      /**
       * \brief density
       */
      double rho;

      /**
       * \brief cross sectional area 
       */
      double A;

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
       * \brief gravitation
       */
      fmatvec::Vec g;

      /**
       * \brief strain damping
       */
      double cEps0D, cEps1D, cEps2D;

      /**
       * \brief shear correction factors
       */
      double sigma1, sigma2;

      /**
       * \brief global system description 
       */
      fmatvec::SymMat M;
      fmatvec::Vec h;

      /**
       * \brief matrices for implicit integration 
       */
      fmatvec::SqrMat dhdq, dhdu;

      /**
       * \brief state at Lagrangian coordinate
       */
      fmatvec::Vec X;

      /**
       * \brief Cardan-object 
       */
      CardanPtr ag;

      FiniteElement1s21CosseratTranslation(); // standard constructor
      FiniteElement1s21CosseratTranslation(const FiniteElement1s21CosseratTranslation&); // copy constructor
      FiniteElement1s21CosseratTranslation& operator=(const FiniteElement1s21CosseratTranslation&); // assignment operator
  };

  inline const fmatvec::SymMat& FiniteElement1s21CosseratTranslation::getM() const { return M; }
  inline const fmatvec::Vec& FiniteElement1s21CosseratTranslation::geth() const { return h; }
  inline const fmatvec::SqrMat& FiniteElement1s21CosseratTranslation::getdhdq() const { return dhdq; }
  inline const fmatvec::SqrMat& FiniteElement1s21CosseratTranslation::getdhdu() const { return dhdu; }
  inline int FiniteElement1s21CosseratTranslation::getqSize() const { return 9; }
  inline int FiniteElement1s21CosseratTranslation::getuSize() const { return 9; }
  inline void  FiniteElement1s21CosseratTranslation::computedhdz(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) { throw MBSim::MBSimError("Error(FiniteElement1s21CosseratTranslation::computedhdz): Not implemented"); }
  inline fmatvec::Vec FiniteElement1s21CosseratTranslation::computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s21CosseratTranslation::computePosition): Not implemented!"); }
  inline fmatvec::SqrMat FiniteElement1s21CosseratTranslation::computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s21CosseratTranslation::computeOrientation): Not implemented!"); }
  inline fmatvec::Vec FiniteElement1s21CosseratTranslation::computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s21CosseratTranslation::computeVelocity): Not implemented!"); }
  inline fmatvec::Vec FiniteElement1s21CosseratTranslation::computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw MBSim::MBSimError("ERROR (FiniteElement1s21CosseratTranslation::computeAngularVelocity): Not implemented!"); }
  inline fmatvec::Mat FiniteElement1s21CosseratTranslation::computeJacobianOfMotion(const fmatvec::Vec& qG,const MBSim::ContourPointData& data) { return computeJXqG(qG,data.getLagrangeParameterPosition()(0)); }
  inline void FiniteElement1s21CosseratTranslation::setShearCorrectionFactors(double sigma1_, double sigma2_) { sigma1 = sigma1_; sigma2 = sigma2_; }
  inline double FiniteElement1s21CosseratTranslation::getl0() const { return l0; }
  inline fmatvec::Mat FiniteElement1s21CosseratTranslation::computeJXqG(const fmatvec::Vec& qG,double x) { throw MBSim::MBSimError("ERROR (FiniteElement1s21CosseratTranslation::computeJXqG): Not implemented!"); }

}

#endif /* _FINITE_ELEMENT_1S_21_COSSERAT_TRANSLATION_H_ */
