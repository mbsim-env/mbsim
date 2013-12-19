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
 * Contact: thorsten.schindler@mytum.de
 */

#ifndef _FLEXIBLE_BAND_H_
#define _FLEXIBLE_BAND_H_

#include "mbsimFlexibleBody/contours/contour1s_flexible.h"
#include "mbsim/contour.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include "mbsimFlexibleBody/contours/neutral_contour/contour_1s_neutral_factory.h"
#endif

namespace MBSimFlexibleBody {

  /**
   * \brief flexible band contour for spatial curves
   * \author Thorsten Schindler
   * \author Roland Zander
   * \date 2009-04-17 initial commit kernel_dev (Thorsten Schindler)
   * \date 2009-07-10 calculation of Jacobian of Translation for Contours (Thorsten Schindler)
   */ 
  class FlexibleBand : public Contour1sFlexible {	
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      FlexibleBand(const std::string& name);
      FlexibleBand(const std::string& name, bool openStructure_);

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBand"; }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void init(MBSim::InitStage stage);
      virtual void updateKinematicsForFrame(MBSim::ContourPointData& cp, MBSim::FrameFeature ff);   
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &cp, int j = 0);
      virtual fmatvec::Vec3 computePosition(MBSim::ContourPointData &cp) { updateKinematicsForFrame(cp, MBSim::position_cosy); return cp.getFrameOfReference().getPosition(); }
      virtual fmatvec::Vec3 computeVelocity(MBSim::ContourPointData &cp) { updateKinematicsForFrame(cp, MBSim::velocity_cosy); return cp.getFrameOfReference().getVelocity(); }
      /***************************************************/
      
      /* INHERITED INTERFACE OF CONTOURCONTINUUM */
//      virtual void computeRootFunctionPosition(MBSim::ContourPointData &cp) { Contour1sFlexible::updateKinematicsForFrame(cp, MBSim::position); }
//      virtual void computeRootFunctionFirstTangent(MBSim::ContourPointData &cp) { Contour1sFlexible::updateKinematicsForFrame(cp, MBSim::firstTangent); }
      // they are the same as the original ones.
//      virtual void computeRootFunctionPosition(MBSim::ContourPointData &cp) { neutral->updateKinematicsForFrame(cp, MBSim::position); }
//      virtual void computeRootFunctionFirstTangent(MBSim::ContourPointData &cp) { neutral->updateKinematicsForFrame(cp, MBSim::firstTangent); }
      /***************************************************/
      
      /* GETTER / SETTER */
      void setCn(const fmatvec::Vec& Cn_);   
      void setWidth(double width_);
      
      /*!
       * \brief set normal distance of band surface to fibre of reference of one dimensional continuum
       * \param normal distance
       */
      void setNormalDistance(double nDist_);
      
      /*!
       * \brief get normal distance of band surface to fibre of reference of one dimensional continuum
       * \return normal distance
       */
      double getNormalDistance() {return nDist;};
      double getWidth() const; 

      bool isOpenStructure() const { return openStructure; }

#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpineExtrusion(OpenMBV::SpineExtrusion* body, Contour1sNeutralFactory * openMBVneutralFibre_) { openMBVBody=body; openMBVNeturalFibre = openMBVneutralFibre_;}
#endif

#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group* getOpenMBVGrp() { return openMBVGrp; }
      OpenMBV::Body* getOpenMBVBody() { return openMBVBody; }
#endif
      /***************************************************/

    private:
      /**
       * \brief normal of flexible band with respect to referencing neutral fibre (outward normal = (n b)*Cn)
       */
      fmatvec::Vec Cn;

      /** 
       * \brief width of flexible band
       */
      double width;

      /**
       * \brief distance from the referencing neutral fibre in direction of given normal
       */
      double nDist; 

      /**
       * \brief open or closed beam structure
       */
      bool openStructure;

#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Body* openMBVBody;
      OpenMBV::Group* openMBVGrp;
#endif

#ifdef HAVE_OPENMBVCPPINTERFACE
      /*!
       * \brief contour for the spine extrusion
       */
      Contour1sNeutralFactory* openMBVNeturalFibre;
#endif

  };

  inline void FlexibleBand::setWidth(double width_) { width = width_; }
  inline void FlexibleBand::setNormalDistance(double nDist_) { nDist = nDist_; }
  inline double FlexibleBand::getWidth() const { return width; } 

}

#endif /* _FLEXIBLE_BAND_H_ */

