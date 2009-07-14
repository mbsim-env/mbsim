/* Copyright (C) 2005-2006  Roland Zander
 
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
 * Contact:
 *   rzander@users.berlios.de
 *
 */
#ifndef _SUPERELEMENT_LINEAR_EXTERNAL_H_
#define _SUPERELEMENT_LINEAR_EXTERNAL_H_

#include "fmatvec.h"
#include "discretization_interface.h"
using namespace fmatvec;

namespace MBSim {

  /*! \brief
   * Superelement for linear models from external preprocessing.
   */
  class SuperElementLinearExternal : public DiscretizationInterface
  {
    protected:
      int warnLevel;
      SymMat M;
      Vec h;

      /** constant stiffness matrix \f$\vK\f$*/ 
      SqrMat K;
      /** constant damping matrix \f$\vD\f$, see setProportionalDamping()*/ 
      SqrMat D; 
      /** constant for damping matrix, see setProportionalDamping*/ 
      double alpha;
      /** constant for damping matrix, see setProportionalDamping*/ 
      double  beta;

      /** container holding constant JACOBIAN-matrizes of all Port s and Contour s */
      vector<Mat> J;
      /** container holding undeformed positions in body coordinate system of all Port s and Contour s */
      vector<Vec> KrP;

    public:
      SuperElementLinearExternal(int warnLevel_=0);
	  void init();

	  /*! set constant mass matrix \f$\vM\f$
	   * \param M mass matrix
	   * */
	  void setM(const SymMat &M_);
	  /*! set constant stiffness matrix \f$\vK\f$
	   * \param K stiffness matrix
	   * */
	  void setK(const SqrMat &K_);
	  /*! set coefficients \f$\alpha\f$ and \f$\beta\f$ for proportional damping:
	   * constant damping matrix \f$\vD\f$ proportional to mass and stiffness
       * \f[ \vD = \alpha * \vM + \beta*\vK \f]
 	   * \param alpha proportional coefficient for mass matrix
 	   * \param beta  proportional coefficient for stiffness matrix
	   * */
	  void setProportionalDamping(double alpha_,double beta_) { alpha=alpha_; beta=beta_; }

	  SymMat getMassMatrix()   const {return M;}
	  Vec    getGeneralizedForceVector()   const {return h;}

	  SqrMat getJacobianForImplicitIntegrationRegardingPosition() const {return SqrMat(-K);}    
	  SqrMat getJacobianForImplicitIntegrationRegardingVelocity() const {return SqrMat(-D);}
	  int getSizeOfPositions()  const {return K.size();}
	  int getSizeOfVelocities()  const {return M.size();}

      /*! 
       * update \f$\vh= -(\vK \vq + \vD \vu)\f$, \f$\vM\f$ is constant
       */
      inline void computeEquationsOfMotion(const Vec& qElement,const Vec& uElement) { h = - K * qElement - D * uElement; }
      double computeKineticEnergy(const fmatvec::Vec& q,const fmatvec::Vec& u) { return 0.5*trans(u)*M*u;}
      double computeGravitationalEnergy(const fmatvec::Vec& q) { return 0.0;}
      double computeElasticEnergy(const fmatvec::Vec& q) { return 0.5*trans(q)*K*q;}

	  Vec computeTranslationalVelocity       (const Vec&q,const Vec&u,const ContourPointData& cp) { return trans(computeJacobianOfMinimalRepresentationRegardingPhysics(q,cp))*u;}
	  Vec computeAngularVelocity    (const Vec&q,const Vec&u,const ContourPointData& cp) { return trans(computeJacobianOfMinimalRepresentationRegardingPhysics(q,cp))*u;}
	  Vec computeTranslation      (const Vec&q,const ContourPointData& cp);
	  SqrMat computeOrientation      (const Vec&q,const ContourPointData& cp) {if(warnLevel>0) cout << "WARNING (SuperElementLinearExternal::computeOrientation): Not Implemented" << endl; return SqrMat(0,INIT,0.);}
	  Mat computeJacobianOfMinimalRepresentationRegardingPhysics(const Vec&q,const ContourPointData& cp);

	  ContourPointData addInterface(Mat J_, Vec KrP_);
  };

}

#endif
