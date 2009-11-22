/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: thschindler@users.berlios.de
 */

#ifndef _FINITE_ELEMENT_1S_33_RCM_H_
#define _FINITE_ELEMENT_1S_33_RCM_H_

#include "mbsim/discretization_interface.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/contour_pdata.h"
#include "mbsim/flexible_body/finite_elements/finite_element_1s_33_rcm/weight33RCM.h"
#include "fmatvec.h"

namespace MBSim {

  class RevCardan;
  class Trafo33RCM;

  /**
   * \brief finite element for spatial beam using Redundant Coordinate Method (RCM)
   * \author Thorsten Schindler
   * \date 2009-04-17 initial commit kernel_dev (Thorsten Schindler)
   * \date 2009-07-27 implicit integration (Thorsten Schindler)
   * \todo transform computeState to Position / Velocity / Orientation / AngularVelocity 
   * \todo JacobianOfMotion 
   * \todo computeM 
   */
  class FiniteElement1s33RCM : public MBSim::DiscretizationInterface {
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
       * \param warning level for outputs
       */
      FiniteElement1s33RCM(double l0_,double rho_,double A_,double E_,double G_,double I1_,double I2_,double I0_,const fmatvec::Vec& g_,RevCardan* ag_);		

      /**
       * \brief destructor
       */
      virtual ~FiniteElement1s33RCM();		

      /* INHERITED INTERFACE OF DISCRETIZATIONINTERFACE */
      virtual const fmatvec::SymMat& getM() const;		
      virtual const fmatvec::Vec& geth() const;
      virtual const fmatvec::SqrMat& getdhdq() const;
      virtual const fmatvec::SqrMat& getdhdu() const;
      virtual int getqSize() const;
      virtual int getuSize() const;

      virtual void computeM(const fmatvec::Vec& qG);
      virtual void computeh(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      virtual void computedhdz(const fmatvec::Vec& qG, const fmatvec::Vec& qGt);
      virtual double computeKineticEnergy(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);		
      virtual double computeGravitationalEnergy(const fmatvec::Vec& qG);		
      virtual double computeElasticEnergy(const fmatvec::Vec& qG);

      virtual fmatvec::Vec computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data);
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& qG,const MBSim::ContourPointData& data);
      /*****************************************************/ 

      /* GETTER / SETTER */
      void setGauss(int nGauss);
      void setCurlRadius(double R1,double R2);		
      void setMaterialDamping(double epstD_,double k0D_);		
      void setLehrDamping(double epstL,double k0L);		
      double getl0() const;		
      /*****************************************************/ 

      /**
       * \brief compute state of finite element
       * \param global coordinates
       * \param global velocities
       * \param LAGRANGIAN parameter
       */
      const fmatvec::Vec& computeState(const fmatvec::Vec& qG,const fmatvec::Vec& qGt,double x);

      /**
       * \brief compute JACOBIAN of contact description in global coordinates
       * \param global coordinates
       * \param LAGRANGIAN parameter
       */
      fmatvec::Mat computeJXqG(const fmatvec::Vec& qG,double x);

      /**
       * \brief compute interesting data 
       * \param global coordinates
       * \param global velocities
       */
      fmatvec::Vec computeData(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);

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
       * \brief predefined bending 
       */
      double k10, k20;

      /**
       * \brief prolongational and torsional damping
       */
      double epstD, k0D;

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
       * \brief internal system description 
       */
      fmatvec::SymMat MI;

      /**
       * \brief internal damping matrix
       */
      fmatvec::SqrMat Damp;

      /**
       * \brief beam length powers 
       */
      double l0h2, l0h3;

      /**
       * \brief last Lagrangian coordinate in state calculation 
       */
      double x_Old;

      /**
       * \brief state at Lagrangian coordinate
       */
      fmatvec::Vec X;


      /**
       * \brief global and local state of the last time step 
       */
      fmatvec::Vec qG_Old, qGt_Old;

      /**
       * \brief tolerance for comparison of state with old state 
       */
      double tol_comp;

      /**
       * \brief delta matrices
       */
      fmatvec::Mat drS, drSH;
      fmatvec::RowVec depstil, dk0;

      /**
       * \brief reversed Cardan-object 
       */
      RevCardan *ag;

      /**
       * \brief trafo-object
       */
      Trafo33RCM *tf;

      /**
       * \brief weight-function-object
       */
      Weight33RCM *wt;

      /**
       * \brief compute delta matrix for CP with respect to translation
       */
      void computedrS();

      /**
       * \brief compute delta matrix for elongation
       */
      void computedepstil();
      
      /**
       * \brief compute delta matrix for torsion
       */
      void computedk0();	
  };

  inline void FiniteElement1s33RCM::setGauss(int nGauss) { wt->setGauss(nGauss); }
  inline double FiniteElement1s33RCM::getl0() const { return l0; }
  inline const fmatvec::SymMat& FiniteElement1s33RCM::getM() const { return M; }
  inline const fmatvec::Vec& FiniteElement1s33RCM::geth() const { return h; }
  inline const fmatvec::SqrMat& FiniteElement1s33RCM::getdhdq() const { return dhdq; }
  inline const fmatvec::SqrMat& FiniteElement1s33RCM::getdhdu() const { return dhdu; }
  inline int FiniteElement1s33RCM::getqSize() const { return 16; }
  inline int FiniteElement1s33RCM::getuSize() const { return 16; }
  inline fmatvec::Vec FiniteElement1s33RCM::computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw new MBSimError("ERROR (FiniteElement1s33RCM::computePosition): Not implemented!"); }
  inline fmatvec::SqrMat FiniteElement1s33RCM::computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data) { throw new MBSimError("ERROR (FiniteElement1s33RCM::computeOrientation): Not implemented!"); }
  inline fmatvec::Vec FiniteElement1s33RCM::computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw new MBSimError("ERROR (FiniteElement1s33RCM::computeVelocity): Not implemented!"); }
  inline fmatvec::Vec FiniteElement1s33RCM::computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) { throw new MBSimError("ERROR (FiniteElement1s33RCM::computeAngularVelocity): Not implemented!"); }
  inline fmatvec::Mat FiniteElement1s33RCM::computeJacobianOfMotion(const fmatvec::Vec& qG,const MBSim::ContourPointData& data) { return computeJXqG(qG,data.getLagrangeParameterPosition()(0)); }

}

#endif /* _FINITE_ELEMENT_1S_33_RCM_H_ */

