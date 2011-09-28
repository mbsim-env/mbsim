/* Copyright (C) 2004-2011 MBSim Development Team
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

#ifndef _FLEXIBLE_BODY_1S_33_COSSERAT_H_
#define _FLEXIBLE_BODY_1S_33_COSSERAT_H_

#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsimFlexibleBody/contours/cylinder_flexible.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_cosserat.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#endif

namespace MBSimFlexibleBody {

  /**
   * \brief finite element for spatial beam using Cosserat model
   * \author Christian Käsbauer
   * \author Thomas Cebulla
   * \author Thorsten Schindler
   * \date 2011-09-10 initial commit (Thorsten Schindler)
   * \todo everything
   *
   * Cosserat model based on
   * H. Lang, J. Linn, M. Arnold: Multi-body dynamics simulation of geometrically exact Cosserat rods
   */
  class FlexibleBody1s33Cosserat : public FlexibleBodyContinuum<double> {
    public:

      /**
       * \brief constructor
       * \param name of body
       * \param bool to specify open (cantilever) or closed (ring) structure
       */
      FlexibleBody1s33Cosserat(const std::string &name, bool openStructure); 

      /**
       * \brief destructor
       */
      virtual ~FlexibleBody1s33Cosserat();

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      virtual void BuildElements();
      virtual void GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
      virtual void GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat);
      virtual void GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat);
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff, MBSim::Frame *frame=0);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &data, MBSim::Frame *frame=0);
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(MBSim::InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBody1s33Cosserat"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setNumberElements(int n);   	
      void setLength(double L_);   	
      void setEGModuls(double E_,double G_);    	
      void setDensity(double rho_);	
      void setCrossSectionalArea(double A_);    	
      void setMomentsInertia(double I1_,double I2_,double I0_);
      void setRelaxed(const fmatvec::Vec& relaxed_);
      void setBoundary(fmatvec::Vec bound_orient_1_,fmatvec::Vec bound_orient_2_,fmatvec::Vec bound_ang_vel_1_,fmatvec::Vec bound_ang_vel_2_);
      void setCylinder(double cylinderRadius_);
      void setCuboid(double cuboidBreadth_,double cuboidHeight_);

#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpineExtrusion(OpenMBV::SpineExtrusion* body) { openMBVBody=body; }
#endif

      double getLength() const { return L; }
      /***************************************************/

      /**
       * \brief compute state (positions, angles, velocities, differentiated angles) at Lagrangian coordinate in local FE coordinates
       * \param Lagrangian coordinate
       */
      fmatvec::Vec computeState(double x);

    private:
      /**
       * \brief contours
       */
      CylinderFlexible *cylinder;
      FlexibleBand *top, *bottom, *left, *right;

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
       * \brief open or closed beam structure
       */
      bool openStructure;

      /**
       * \brief initialised FLAG 
       */
      bool initialized;

      /**
       * \brief boundary conditions for first and last finite difference beam element
       */
      fmatvec::Vec bound_orient_1;
      fmatvec::Vec bound_orient_2;
      fmatvec::Vec bound_ang_vel_1;
      fmatvec::Vec bound_ang_vel_2;

      /**
       * \brief ? TODO
       */
      fmatvec::Vec relaxed;

      /**
       * \brief contour data 
       */
      double cuboidBreadth, cuboidHeight, cylinderRadius,;

      FlexibleBody1s33Cosserat(); // standard constructor
      FlexibleBody1s33Cosserat(const FlexibleBody1s33Cosserat&); // copy constructor
      FlexibleBody1s33Cosserat& operator=(const FlexibleBody1s33Cosserat&); // assignment operator

      /**
       * \brief detect current finite element
       * \param global parametrisation
       * \param local parametrisation
       * \param finite element number
       */
      void BuildElement(const double& sGlobal, double& sLocal, int& currentElement);
  };

  inline void FlexibleBody1s33Cosserat::updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff, MBSim::Frame *frame) { throw MBSim::MBSimError("ERROR(FlexibleBody1s33Cosserat::updateKinematicsForFrame): Not implemented!"); }
  inline void FlexibleBody1s33Cosserat::updateJacobiansForFrame(MBSim::ContourPointData &data, MBSim::Frame *frame) { throw MBSim::MBSimError("ERROR(FlexibleBody1s33Cosserat::updateJacobiansForFrame): Not implemented!"); }
  inline void FlexibleBody1s33Cosserat::setLength(double L_) { L = L_; }
  inline void FlexibleBody1s33Cosserat::setEGModuls(double E_,double G_) { E = E_; G = G_; }    	
  inline void FlexibleBody1s33Cosserat::setDensity(double rho_) { rho = rho_;}     	
  inline void FlexibleBody1s33Cosserat::setCrossSectionalArea(double A_) { A = A_; }    	
  inline void FlexibleBody1s33Cosserat::setMomentsInertia(double I1_, double I2_, double I0_) { I1 = I1_; I2 = I2_; I0 = I0_; }    	
  inline void FlexibleBody1s33Cosserat::setRelaxed(const fmatvec::Vec& relaxed_) { relaxed = relaxed_; }
  inline void FlexibleBody1s33Cosserat::setBoundary(fmatvec::Vec bound_orient_1_, fmatvec::Vec bound_orient_2_, fmatvec::Vec bound_ang_vel_1_, fmatvec::Vec bound_ang_vel_2_){ bound_orient_1 = bound_orient_1_; bound_orient_2 = bound_orient_2_; bound_ang_vel_1 = bound_ang_vel_1_ ; bound_ang_vel_2 = bound_ang_vel_2_; }
  inline void FlexibleBody1s33Cosserat::setCylinder(double cylinderRadius_) { cylinderRadius = cylinderRadius; }
  inline void FlexibleBody1s33Cosserat::setCuboid(double cuboidBreadth_,double cuboidHeight_) { cuboidBreadth = cuboidBreadth_; cuboidHeight = cuboidHeight_; }

}

#endif /* _FLEXIBLE_BODY_1S_33_COSSERAT_H_ */

