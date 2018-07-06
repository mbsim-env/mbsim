/* Copyright (C) 2004-2016 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _KINEMATIC_COORDINATES_OBSERVER_H__
#define _KINEMATIC_COORDINATES_OBSERVER_H__

#include "mbsim/observers/observer.h"

#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  class Frame;

  class KinematicCoordinatesObserver : public Observer {
    private:
      Frame *frame, *frameOfReference;
      std::shared_ptr<OpenMBVArrow> ombvPositionArrow, ombvVelocityArrow, ombvAccelerationArrow;
      std::shared_ptr<OpenMBV::Group> openMBVPosGrp, openMBVVelGrp, openMBVAccGrp;
      std::shared_ptr<OpenMBV::Arrow> openMBVPosition, openMBVVelocity, openMBVAcceleration, openMBVXPosition, openMBVYPosition, openMBVZPosition, openMBVXVelocity, openMBVYVelocity, openMBVZVelocity, openMBVXAcceleration, openMBVYAcceleration, openMBVZAcceleration;
      std::string saved_frame, saved_frameOfReference;
#ifndef SWIG
      double (KinematicCoordinatesObserver::*evalOMBVPositionColorRepresentation[2])();
      double (KinematicCoordinatesObserver::*evalOMBVVelocityColorRepresentation[2])();
      double (KinematicCoordinatesObserver::*evalOMBVAccelerationColorRepresentation[2])();
#endif
      double evalNone() { return 0.5; }
      double evalAbsolutePosition();
      double evalAbsoluteVelocitiy();
      double evalAbsoluteAcceleration();

    public:
      KinematicCoordinatesObserver(const std::string &name="");

      void setFrame(Frame *frame_) { frame = frame_; }
      void setFrameOfReference(Frame *frameOfReference_) { frameOfReference = frameOfReference_; }

      void init(InitStage stage, const InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void plot() override;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVPosition, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::fromPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        ombvPositionArrow = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency));
      }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVVelocity, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::fromPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        ombvVelocityArrow = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency));
      }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAcceleration, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::fromPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        ombvAccelerationArrow = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency));
      }
  };

}

#endif
