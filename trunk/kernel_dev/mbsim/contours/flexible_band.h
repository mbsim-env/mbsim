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

#ifndef _FLEXIBLE_BAND_H_
#define _FLEXIBLE_BAND_H_

#include "mbsim/contours/contour1s_flexible.h"
#include "mbsim/contour.h"

namespace MBSim {

  /**
   * \brief flexible band contour for spatial curves
   * \author Thorsten Schindler
   * \author Roland Zander
   * \date 2009-04-17 initial commit kernel_dev (Thorsten Schindler)
   * \date 2009-07-10 calculation of Jacobian of Translation for Contours (Thorsten Schindler)
   */ 
  class FlexibleBand : public MBSim::Contour1sFlexible {	
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      FlexibleBand(const std::string& name);

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(ContourPointData& cp, FrameFeature ff);   
      virtual void updateJacobiansForFrame(ContourPointData &cp); 
      virtual fmatvec::Vec computePosition(ContourPointData &cp) { updateKinematicsForFrame(cp,position_cosy); return cp.getFrameOfReference().getPosition(); }
      virtual fmatvec::Vec computeVelocity(ContourPointData &cp) { updateKinematicsForFrame(cp,velocity_cosy); return cp.getFrameOfReference().getVelocity(); }
      /***************************************************/
      
      /* INHERITED INTERFACE OF CONTOURCONTINUUM */
      virtual void computeRootFunctionPosition(ContourPointData &cp) { Contour1sFlexible::updateKinematicsForFrame(cp,position); }
      virtual void computeRootFunctionFirstTangent(ContourPointData &cp) { Contour1sFlexible::updateKinematicsForFrame(cp,firstTangent); }
      /***************************************************/
      
      /* GETTER / SETTER */
      void setCn(const fmatvec::Vec& Cn_);   
      void setWidth(double width_);
      void setNormalDistance(double nDist_);
      double getWidth() const; 
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
  };

  inline void FlexibleBand::setWidth(double width_) { width = width_; }
  inline void FlexibleBand::setNormalDistance(double nDist_) { nDist = nDist_; }
  inline double FlexibleBand::getWidth() const { return width; } 

}

#endif /* _FLEXIBLE_BAND_H_ */

