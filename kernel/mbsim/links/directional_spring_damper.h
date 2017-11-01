/* Copyright (C) 2004-2009 MBSim Development Team
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

#ifndef _DIRECTIONAL_SPRING_DAMPER_H_
#define _DIRECTIONAL_SPRING_DAMPER_H_

#include "mbsim/links/floating_frame_link.h"
#include "mbsim/functions/function.h"

#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace MBSim {

  /** \brief A spring damper force law.
   * This class connects two frames and applies a force in it, which depends in the
   * distance and relative velocity between the two frames.
   */
  class DirectionalSpringDamper : public FloatingFrameLink {
    protected:
      double dist;
      Function<double(double,double)> *func;
      double l0;
      std::shared_ptr<OpenMBV::CoilSpring> coilspringOpenMBV;
    public:
      DirectionalSpringDamper(const std::string &name="");
      ~DirectionalSpringDamper();

      void updatePositions(Frame *frame);
      void updateGeneralizedPositions();
      void updateGeneralizedVelocities();
      void updatelaF();

      /*INHERITED INTERFACE OF LINK*/
      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      bool isSingleValued() const { return true; }
      void init(InitStage stage, const InitConfigSet &config);
      /*****************************/

      /** \brief Set function for the force calculation.
       * The first input parameter to that function is the distance relative to the unloaded length.
       * The second input parameter to that function is the relative velocity.       
       * The return value of that function is used as the force of the SpringDamper.
       */
      void setForceFunction(Function<double(double,double)> *func_) {
        func=func_;
        func->setParent(this);
        func->setName("Force");
      }

      /** \brief Set unloaded length. */
      void setUnloadedLength(double l0_) { l0 = l0_; }

      /**
       * \param local force direction represented in first frame
       */
      void setForceDirection(const fmatvec::Vec3 &dir) { forceDir=dir/nrm2(dir); }

      void plot();
      void initializeUsingXML(xercesc::DOMElement *element);

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (numberOfCoils,(int),3)(springRadius,(double),1)(crossSectionRadius,(double),-1)(nominalLength,(double),-1)(type,(OpenMBV::CoilSpring::Type),OpenMBV::CoilSpring::tube)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(minimalColorValue,(double),0)(maximalColorValue,(double),1))) {
        OpenMBVCoilSpring ombv(springRadius,crossSectionRadius,1,numberOfCoils,nominalLength,type,diffuseColor,transparency,minimalColorValue,maximalColorValue);
        coilspringOpenMBV=ombv.createOpenMBV();
      }
  };

}

#endif
