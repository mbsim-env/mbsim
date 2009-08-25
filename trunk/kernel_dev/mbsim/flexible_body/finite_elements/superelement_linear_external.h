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
 * Contact: rzander@users.berlios.de
 */

#ifndef _SUPERELEMENT_LINEAR_EXTERNAL_H_
#define _SUPERELEMENT_LINEAR_EXTERNAL_H_

#include "fmatvec.h"
#include "mbsim/discretization_interface.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/contour_pdata.h"

namespace MBSim {

  /*! 
   * \brief superelement for linear models from external preprocessing.
   * \author Roland Zander
   * \date 2009-05-22 some references added (Thorsten Schindler)
   * \date 2009-07-16 fixed proportional damping (Thorsten Schindler)
   * \date 2009-07-23 implicit integration (Thorsten Schindler)
   */
  class SuperElementLinearExternal : public DiscretizationInterface {
    public:
      SuperElementLinearExternal();
	  void init(InitStage stage);

	  /*! set constant mass matrix \f$\vM\f$
	   * \param M mass matrix
	   * */
	  void setM(const fmatvec::SymMat &M_);
	  
      /*! set constant stiffness matrix \f$\vK\f$
	   * \param K stiffness matrix
	   * */
	  void setK(const fmatvec::SqrMat &K_);
	  
      /*! set coefficients \f$\alpha\f$ and \f$\beta\f$ for proportional damping:
	   * constant damping matrix \f$\vD\f$ proportional to mass and stiffness
       * \f[ \vD = \alpha * \vM + \beta*\vK \f]
 	   * \param alpha proportional coefficient for mass matrix
 	   * \param beta  proportional coefficient for stiffness matrix
	   * */
	  void setProportionalDamping(double alpha_,double beta_) { alpha=alpha_; beta=beta_; }

      const fmatvec::SymMat& getM() const { return M; }
      const fmatvec::Vec&    geth() const { return h; }

      const fmatvec::SqrMat& getdhdq() const { return Dhq; }    
      const fmatvec::SqrMat& getdhdu() const { return Dhqp; }
	  int getqSize() const { return K.size(); }
	  int getuSize() const { return M.size(); }

      void computeM(const fmatvec::Vec& qElement) {}
      /*! 
       * update \f$\vh= -(\vK \vq + \vD \vu)\f$, \f$\vM\f$ is constant
       */
      void computeh(const fmatvec::Vec& qElement,const fmatvec::Vec& uElement) { h = - K * qElement - D * uElement; }
      void computedhdz(fmatvec::Vec& qElement,fmatvec::Vec& uElement) {}
      double computeKineticEnergy(const fmatvec::Vec& q,const fmatvec::Vec& u) { return 0.5*trans(u)*M*u;}
      double computeGravitationalEnergy(const fmatvec::Vec& q) { return 0.0;}
      double computeElasticEnergy(const fmatvec::Vec& q) { return 0.5*trans(q)*K*q;}

      fmatvec::Vec computeVelocity(const fmatvec::Vec&q,const fmatvec::Vec&u,const ContourPointData& cp) { return trans(computeJacobianOfMotion(q,cp))*u;}
      fmatvec::Vec computeAngularVelocity(const fmatvec::Vec&q,const fmatvec::Vec&u,const ContourPointData& cp) { return trans(computeJacobianOfMotion(q,cp))*u;}
      fmatvec::Vec computePosition(const fmatvec::Vec&q,const ContourPointData& cp);
      fmatvec::SqrMat computeOrientation(const fmatvec::Vec&q,const ContourPointData& cp) { throw new MBSimError("ERROR(SuperElementLinearExternal::computeOrientation): Not Implemented");}
      fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec&q,const ContourPointData& cp);

	  ContourPointData addInterface(fmatvec::Mat J_, fmatvec::Vec KrP_);
    
    protected:
      /** constant mass matrix \f$\vM\f$*/ 
      fmatvec::SymMat M;
      /** right hand side */
      fmatvec::Vec h;
      /** constant stiffness matrix \f$\vK\f$*/ 
      fmatvec::SqrMat K;
      /** constant damping matrix \f$\vD\f$, see setProportionalDamping()*/ 
      fmatvec::SqrMat D; 
      /** constant for damping matrix, see setProportionalDamping*/ 
      double alpha;
      /** constant for damping matrix, see setProportionalDamping*/ 
      double beta;
      /** constant Jacobians for implicit integration */
      fmatvec::SqrMat Dhq, Dhqp;
      /** container holding constant JACOBIAN-matrizes of all Frames and Contours */
      std::vector<fmatvec::Mat> J;
      /** container holding undeformed positions in body coordinate system of all Frames and Contours */
      std::vector<fmatvec::Vec> KrP;
  };

}

#endif /* _SUPERELEMENT_LINEAR_EXTERNAL_H_ */

