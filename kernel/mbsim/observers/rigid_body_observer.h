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

#ifndef _RIGID_BODY_OBSERVER_H__
#define _RIGID_BODY_OBSERVER_H__

#include "mbsim/observers/observer.h"

#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace MBSim {

  BOOST_PARAMETER_NAME(sideOfInteraction)

  class RigidBody;

  class RigidBodyObserver: public Observer {
    public:
      enum SideOfInteraction {
        action=0,
        reaction,
        both,
        unknown
      };

    private:
      RigidBody* body;
      SideOfInteraction sideOfForceInteraction{action}, sideOfMomentInteraction{action};
      Frame *frameOfReference;
      std::shared_ptr<OpenMBVArrow> ombvForce, ombvMoment;
      std::vector<std::shared_ptr<OpenMBV::Arrow>> FArrow, MArrow;
      std::shared_ptr<OpenMBV::Arrow> FWeight, openMBVAxisOfRotation, openMBVMomentum, openMBVAngularMomentum, openMBVDerivativeOfMomentum, openMBVDerivativeOfAngularMomentum;
      std::string saved_body;
      std::string saved_frameOfReference;

    public:
      RigidBodyObserver(const std::string &name="");
      void setRigidBody(RigidBody *body_) { body = body_; } 
      void setFrameOfReference(Frame *frameOfReference_) { frameOfReference = frameOfReference_; }

      void init(InitStage stage, const InitConfigSet &config);
      void initializeUsingXML(xercesc::DOMElement *element);
      void plot();

     /** \brief Visualize the weight */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVWeight, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(scaleLength,scaleSize,OpenMBV::Arrow::toHead,referencePoint,minimalColorValue,maximalColorValue,diffuseColor,transparency);
        FWeight=ombv.createOpenMBV();
      }

      /** \brief Visualize the joint force */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVJointForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(sideOfInteraction,(SideOfInteraction),action))) {
        sideOfForceInteraction = sideOfInteraction;
        ombvForce = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBV::Arrow::toHead,referencePoint,minimalColorValue,maximalColorValue,diffuseColor,transparency));
      }

      /** \brief Visualize the joint moment */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVJointMoment, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(sideOfInteraction,(SideOfInteraction),action))) {
        sideOfMomentInteraction = sideOfInteraction;
        ombvMoment = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBV::Arrow::toDoubleHead,referencePoint,minimalColorValue,maximalColorValue,diffuseColor,transparency));
      }

      /** \brief Visualize the center of rotation */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAxisOfRotation, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(scaleLength,scaleSize,OpenMBV::Arrow::toHead,referencePoint,minimalColorValue,maximalColorValue,diffuseColor,transparency);
        openMBVAxisOfRotation=ombv.createOpenMBV();
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMomentum, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(scaleLength,scaleSize,OpenMBV::Arrow::toHead,referencePoint,minimalColorValue,maximalColorValue,diffuseColor,transparency);
        openMBVMomentum=ombv.createOpenMBV();
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAngularMomentum, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(scaleLength,scaleSize,OpenMBV::Arrow::toDoubleHead,referencePoint,minimalColorValue,maximalColorValue,diffuseColor,transparency);
        openMBVAngularMomentum=ombv.createOpenMBV();
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVDerivativeOfMomentum, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(scaleLength,scaleSize,OpenMBV::Arrow::toHead,referencePoint,minimalColorValue,maximalColorValue,diffuseColor,transparency);
        openMBVDerivativeOfMomentum=ombv.createOpenMBV();
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVDerivativeOfAngularMomentum, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(scaleLength,scaleSize,OpenMBV::Arrow::toDoubleHead,referencePoint,minimalColorValue,maximalColorValue,diffuseColor,transparency);
        openMBVDerivativeOfAngularMomentum=ombv.createOpenMBV();
      }
  };

}  

#endif
