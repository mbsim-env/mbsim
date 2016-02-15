/* Copyright (C) 2004-2012 MBSim Development Team
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
 * Contact: ghorst@amm.mw.tum.de
 */

#ifndef _ISOTROPICROTATIONALSPRINGDAMPER_H_
#define _ISOTROPICROTATIONALSPRINGDAMPER_H_

#include "mbsim/floating_frame_link.h"

namespace MBSim {

  /** 
   * \brief Isotropic rotational spring damper force law.
   * This class connects two frames and applies a torque which depends on the
   * relative rotation and velocity between the two frames.
   * Not considered: torsion around the first axis / rotation more than 180°
   * 
   * \author Gerald Horst
   * \author Thorsten Schindler
   * \date 2012-03-21 initial commit (Thorsten Schindler)
   */
  class IsotropicRotationalSpringDamper : public FloatinFrameLink {
    public:
      /**
       * \brief constructor
       * \param name of isotropic roational spring damper
       */
      IsotropicRotationalSpringDamper(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~IsotropicRotationalSpringDamper();

      virtual void updateGeneralizedPositions(double t);
      virtual void updateGeneralizedVelocities(double t);
      /***************************************************/

      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      virtual bool isActive() const { return true; }
      virtual bool gActiveChanged() { return false; }
      virtual bool isSingleValued() const { return true; }
      /***************************************************/

      /* SETTER */
      void setParameters(double c_, double d_, double alpha0_) {
        c = c_;
        d = d_;
        alpha0 = alpha0_;
      }

      void setMomentDirection(const fmatvec::Mat3xV& md);

    private:
      /**
       * \brief stiffness, damping, relaxed angle
       */
      double c, d, alpha0;
  };

}

#endif /* _ISOTROPICROTATIONALSPRINGDAMPER_H_ */

