/* Copyright (C) 2004-2015 MBSim Development Team
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

#ifndef _RIGID_BODY_SYSTEM_OBSERVER_H__
#define _RIGID_BODY_SYSTEM_OBSERVER_H__

#include "mbsim/observers/observer.h"

#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  class RigidBody;
  class Frame;

  class RigidBodySystemObserver : public Observer {
    private:
      std::vector<RigidBody*> body;
      Frame *frameOfReference;
      std::shared_ptr<OpenMBVArrow> ombvPosition, ombvVelocity, ombvAcceleration, ombvWeight, ombvMomentum, ombvAngularMomentum, ombvDerivativeOfMomentum, ombvDerivativeOfAngularMomentum;
      std::shared_ptr<OpenMBV::Arrow> openMBVPosition, openMBVVelocity, openMBVAcceleration, openMBVWeight, openMBVMomentum, openMBVAngularMomentum, openMBVDerivativeOfMomentum, openMBVDerivativeOfAngularMomentum;
      std::vector<std::string> saved_body;
      std::string saved_frameOfReference;

    public:
      RigidBodySystemObserver(const std::string &name="");
      void addRigidBody(RigidBody *body_) { body.push_back(body_); }
      void setFrameOfReference(Frame *frameOfReference_) { frameOfReference = frameOfReference_; }

      void init(InitStage stage, const InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void plot() override;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVPosition, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::fromPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvPosition = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVVelocity, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::fromPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvVelocity = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAcceleration, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::fromPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvAcceleration = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVWeight, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::fromPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvWeight = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMomentum, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::fromPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvMomentum = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAngularMomentum, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::fromPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvAngularMomentum = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBVArrow::toDoubleHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVDerivativeOfMomentum, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::fromPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvDerivativeOfMomentum = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVDerivativeOfAngularMomentum, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::fromPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvDerivativeOfAngularMomentum = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBVArrow::toDoubleHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }

  };

}

#endif
