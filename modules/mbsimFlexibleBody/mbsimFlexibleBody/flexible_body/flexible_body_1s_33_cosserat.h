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
   * \author Thorsten Schindler
   * \author Christian Käsbauer
   * \author Thomas Cebulla
   * \date 2011-09-10 initial commit (Thorsten Schindler)
   * \data 2011-10-08 basics derived and included (Thorsten Schindler)
   * \todo understand boundary conditions TODO
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
      virtual void facLLM();
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateh(double t);

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
      void setCurlRadius(double R1_,double R2_);
      void setMaterialDamping(double cEps0D_);
      void setBoundary(fmatvec::Vec bound_ang_start_,fmatvec::Vec bound_ang_end_,fmatvec::Vec bound_ang_vel_start_,fmatvec::Vec bound_ang_vel_end_);
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
      fmatvec::Vec computeState(double s);

    private:
      /** 
       * \brief stl-vector of finite elements for rotation grid
       */
      std::vector<MBSim::DiscretizationInterface*> rotationDiscretization;

      /** 
       * \brief stl-vector of finite element positions for rotation grid
       */
      std::vector<fmatvec::Vec> qRotationElement;

      /** 
       * \brief stl-vector of finite element wise velocities for rotation grid
       */
      std::vector<fmatvec::Vec> uRotationElement;

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
       * \brief radius of undeformed shape
       */
      double R1, R2;

      /**
       * \brief elongational damping 
       */
      double cEps0D;

      /**
       * \brief open or closed beam structure
       */
      bool openStructure;

      /**
       * \brief initialised FLAG 
       */
      bool initialised;

      /**
       * \brief boundary conditions for first and last finite difference beam element
       * TODO
       */
      fmatvec::Vec bound_ang_start;
      fmatvec::Vec bound_ang_end;
      fmatvec::Vec bound_ang_vel_start;
      fmatvec::Vec bound_ang_vel_end;

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
      
      /**
       * \brief initialize translational part of mass matrix and calculate Cholesky decomposition
       */
      void initM();
  };

  inline void FlexibleBody1s33Cosserat::updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff, MBSim::Frame *frame) { throw MBSim::MBSimError("ERROR(FlexibleBody1s33Cosserat::updateKinematicsForFrame): Not implemented!"); }
  inline void FlexibleBody1s33Cosserat::updateJacobiansForFrame(MBSim::ContourPointData &data, MBSim::Frame *frame) { throw MBSim::MBSimError("ERROR(FlexibleBody1s33Cosserat::updateJacobiansForFrame): Not implemented!"); }
  inline void FlexibleBody1s33Cosserat::setLength(double L_) { L = L_; }
  inline void FlexibleBody1s33Cosserat::setEGModuls(double E_,double G_) { E = E_; G = G_; }    	
  inline void FlexibleBody1s33Cosserat::setDensity(double rho_) { rho = rho_;}     	
  inline void FlexibleBody1s33Cosserat::setCrossSectionalArea(double A_) { A = A_; }    	
  inline void FlexibleBody1s33Cosserat::setMomentsInertia(double I1_, double I2_, double I0_) { I1 = I1_; I2 = I2_; I0 = I0_; }    	
  inline void FlexibleBody1s33Cosserat::setCurlRadius(double R1_,double R2_) { R1 = R1_; R2 = R2_; if(initialised) for(int i=0;i<Elements;i++) static_cast<FiniteElement1s33Cosserat*>(discretization[i])->setCurlRadius(R1,R2); }    	
  inline void FlexibleBody1s33Cosserat::setMaterialDamping(double cEps0D_) { cEps0D = cEps0D_; if(initialised) for(int i=0;i<Elements;i++) static_cast<FiniteElement1s33Cosserat*>(discretization[i])->setMaterialDamping(Elements*cEps0D); }
  inline void FlexibleBody1s33Cosserat::setBoundary(fmatvec::Vec bound_ang_start_, fmatvec::Vec bound_ang_end_, fmatvec::Vec bound_ang_vel_start_, fmatvec::Vec bound_ang_vel_end_){ bound_ang_start = bound_ang_start_; bound_ang_end = bound_ang_end_; bound_ang_vel_start = bound_ang_vel_start_ ; bound_ang_vel_end = bound_ang_vel_end_; }
  inline void FlexibleBody1s33Cosserat::setCylinder(double cylinderRadius_) { cylinderRadius = cylinderRadius; }
  inline void FlexibleBody1s33Cosserat::setCuboid(double cuboidBreadth_,double cuboidHeight_) { cuboidBreadth = cuboidBreadth_; cuboidHeight = cuboidHeight_; }

}

#endif /* _FLEXIBLE_BODY_1S_33_COSSERAT_H_ */

