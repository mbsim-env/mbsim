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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _ISOTROPIC_ROTATIONAL_SPRING_DAMPER_H_
#define _ISOTROPIC_ROTATIONAL_SPRING_DAMPER_H_

#include "mbsim/links/fixed_frame_link.h"
#include "mbsim/functions/function.h"

namespace MBSim {

  /** 
   * \brief Isotropic rotational spring damper.
   * This class connects two frames and applies a torque which depends on the
   * relative rotation and velocity between the two frames.
   * 
   */
  class IsotropicRotationalSpringDamper : public FixedFrameLink {
    public:
      /**
       * \brief constructor
       * \param name of isotropic roational spring damper
       */
      IsotropicRotationalSpringDamper(const std::string &name="");

      /**
       * \brief destructor
       */
      ~IsotropicRotationalSpringDamper();

      void calcSize();

      void updateGeneralizedPositions();
      void updateGeneralizedVelocities();
      void updateForceDirections();
      void updatelaM();

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      bool isSingleValued() const { return true; }
      void init(InitStage stage, const InitConfigSet &config);

      void setElasticMomentFunction(Function<double(double)> *func_) {
        funcE=func_;
        funcE->setParent(this);
        funcE->setName("ElasticMoment");
      }

      void setDisspativeMomentFunction(Function<double(double)> *func_) {
        funcD=func_;
        funcD->setParent(this);
        funcD->setName("DissipativeMoment");
      }

      void initializeUsingXML(xercesc::DOMElement *element);

    private:
      Function<double(double)> *funcE;
      Function<double(double)> *funcD;
      fmatvec::Vec3 n;
  };

}

#endif
