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

#ifndef _FLEXIBLE_BODY_1S_33_RCM_H_
#define _FLEXIBLE_BODY_1S_33_RCM_H_

#include "mbsim/flexible_body.h"
#include "mbsim/flexible_body/finite_elements/finite_element_1s_33_rcm.h"

namespace MBSim {

  class CylinderFlexible;
  class FlexibleBand;

  /**
   * \brief spatial beam using Redundant Coordinate Method (RCM)
   * \author Thorsten Schindler
   * \date 2009-04-17 initial commit kernel_dev (Thorsten Schindler)
   * \todo gyroscopic accelerations TODO
   * \todo inverse kinetics TODO
   */
  class FlexibleBody1s33RCM : public FlexibleBodyContinuum<double> {
    public:
      /**
       * \brief constructor
       * \param name of body
       * \param bool to specify open (cantilever) or closed (ring) structure
       */
      FlexibleBody1s33RCM(const std::string &name,bool openStructure); 
      
      /**
       * \brief destructor
       */
      virtual ~FlexibleBody1s33RCM();

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      virtual std::string getType() const { return "FlexibleBody1s33RCM"; }
      virtual void BuildElements();
      virtual void GlobalMatrixContribution(int n);
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame=0);
      virtual void updateJacobiansForFrame(ContourPointData &data, Frame *frame=0);
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init();
      /***************************************************/

      /* GETTER / SETTER */
      void setGauss(int nGauss);
      void setCylinder(double cylinderRadius_);		
      void setCuboid(double cuboidBreadth_,double cuboidHeight_);
      void setLength(double L_);   	
      void setEGModuls(double E_,double G_);    	
      void setCrossSectionalArea(double A_);    	
      void setMomentsInertia(double I1_,double I2_,double I0_);
      void setDensity(double rho_);	
      void setCurlRadius(double R1_,double R2_);
      void setMaterialDamping(double epstD_,double k0D_);
      void setLehrDamping(double epstL_,double k0L_);
      void setNumberElements(int n);   	
      /***************************************************/

      /**
       * \brief compute state (positions, angles, velocities, differentiated angles) at Lagrangian coordinate in local FE coordinates
       * \param Lagrangian coordinate
       */
      fmatvec::Vec computeState(double x);
      
      /**
       * initialise beam only for giving information with respect to state, number elements, length, (not for simulation)
       */
      void initInfo();

    private:
      /**
       * \brief contours
       */
      CylinderFlexible *cylinder;
      FlexibleBand *top, *bottom, *left, *right;

      /**
       * \brief angle parametrisation
       */
      RevCardan *angle;

      /**
       * \brief number of elements 
       */
      int Elements;

      /**
       * \brief length of entire beam and finite elements
       */
      double L, l0;

      /**
       * \brief elastic modules 
       */
      double E, G;

      /**
       * \brief area of cross-section
       */
      double A;

      /**
       * \brief area moment of inertia 
       */
      double I1, I2, I0;

      /**
       * \brief density 
       */
      double rho;

      /**
       * \brief radius of undeformed shape
       */
      double R1, R2;

      /**
       * \brief damping 
       */
      double epstD, k0D, epstL, k0L;

      /**
       * \brief open or closed structure 
       */
      bool openStructure;

      /**
       * \brief implicit integration 
       */
      bool implicit;

      /**
       * \brief current element number
       */
      int CurrentElement;

      /**
       * \brief Jacobians for implicit integration
       */
      fmatvec::SqrMat JhGqG,JhGqGt; // cannot be initialised in constructor because of unknown size

      /**
       * \brief initialised FLAG 
       */
      bool initialised;

      /**
       * \brief number of Gauss points for rotational kinetic energy 
       */
      int nGauss;

      /**
       * \brief contour data 
       */
      double cylinderRadius, cuboidBreadth, cuboidHeight;

      /**
       * \brief detect current finite element
       * \param global parametrisation
       */
      double BuildElement(double sGlobal);
  };

  inline void FlexibleBody1s33RCM::setGauss(int nGauss_) { nGauss = nGauss_; }
  inline void FlexibleBody1s33RCM::setLength(double L_) { L = L_; }    	
  inline void FlexibleBody1s33RCM::setEGModuls(double E_,double G_) { E = E_; G = G_; }    	
  inline void FlexibleBody1s33RCM::setCrossSectionalArea(double A_) { A = A_; }    	
  inline void FlexibleBody1s33RCM::setMomentsInertia(double I1_,double I2_,double I0_) { I1 = I1_; I2 = I2_; I0 = I0_; }    	
  inline void FlexibleBody1s33RCM::setDensity(double rho_) { rho = rho_;}     	
  inline void FlexibleBody1s33RCM::setCurlRadius(double R1_,double R2_) { R1 = R1_; R2 = R2_; if(initialised) for(int i=0;i<Elements;i++) dynamic_cast<FiniteElement1s33RCM*>(discretization[i])->setCurlRadius(R1,R2); }    	
  inline void FlexibleBody1s33RCM::setMaterialDamping(double epstD_,double k0D_) {epstD = epstD_; k0D = k0D_; if(initialised) for(int i=0;i<Elements;i++) dynamic_cast<FiniteElement1s33RCM*>(discretization[i])->setMaterialDamping(Elements*epstD,Elements*k0D); }   	
  inline void FlexibleBody1s33RCM::setLehrDamping(double epstL_,double k0L_) { epstL = epstL_; k0L = k0L_; if(initialised) for(int i=0;i<Elements;i++) dynamic_cast<FiniteElement1s33RCM*>(discretization[i])->setLehrDamping(Elements*epstL,Elements*k0L); }   	

}

#endif /* _FLEXIBLE_BODY_1S_33_RCM_H_ */

