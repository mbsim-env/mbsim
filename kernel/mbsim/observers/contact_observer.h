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

#ifndef _CONTACT_OBSERVER_H__
#define _CONTACT_OBSERVER_H__

#include "mbsim/observers/single_contact_observer.h"

#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  class Contact;

  class ContactObserver : public Observer {
    protected:
      std::vector<SingleContactObserver> contactObserver;

      Contact* link;
      std::string saved_link;

      /**
       * \brief container of ContactFrames to draw
       */
      std::shared_ptr<OpenMBV::Frame> openMBVContactFrame;

      std::shared_ptr<OpenMBVInteractionArrow> ombvForce, ombvMoment, ombvContact;
      std::shared_ptr<OpenMBVFrictionArrow> ombvFriction;

    public:
      ContactObserver(const std::string &name="");
      void setContact(Contact *link_) { link = link_; }

      void init(InitStage stage, const InitConfigSet &config) override;
      void plot() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void setDynamicSystemSolver(DynamicSystemSolver *sys) override;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVForce, tag, (optional (sideOfInteraction,(OpenMBVInteractionArrow::SideOfInteraction),OpenMBVInteractionArrow::action)(scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        ombvForce = std::shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(sideOfInteraction,scaleLength,scaleSize,OpenMBV::Arrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency));
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMoment, tag, (optional (sideOfInteraction,(OpenMBVInteractionArrow::SideOfInteraction),OpenMBVInteractionArrow::action)(scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(colorRepresentation,(OpenMBVInteractionArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        ombvMoment = std::shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(sideOfInteraction,scaleLength,scaleSize,OpenMBV::Arrow::toDoubleHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency));
      }

      /** 
       * \brief Draw two OpenMBV::Frame's of size 'size' at the contact points if 'enable'==true, otherwise the object is available but disabled.
       * If the contact is closed, then the two contact points are the same on each contour.
       * If the contact is not closed, then the two contact point lie on the contours with minimal distance in between.
       * The x-axis of this frames are orientated to the other frame origin (normal vector).
       */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVContactPoints, tag, (optional (size,(double),1)(offset,(double),1)(transparency,(double),0))) {
        OpenMBVFrame ombv(size,offset,"[-1;1;1]",transparency);
        openMBVContactFrame=ombv.createOpenMBV();
      }

      /** 
       * \brief Sets the OpenMBV::Arrow to be used for drawing the normal force vector.
       * This vector is the force which is applied on the second contour.
       * The reactio (not drawn) is applied on the first contour.
       */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVNormalForce, tag, (optional (sideOfInteraction,(OpenMBVInteractionArrow::SideOfInteraction),OpenMBVInteractionArrow::action)(scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        ombvContact = std::shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(sideOfInteraction,scaleLength,scaleSize,OpenMBV::Arrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency));
      }

      /** 
       * \brief Sets the OpenMBV::Arrow to be used for drawing the friction force vector.
       * This vector is the friction which is applied on the second contour.
       * The reactio (not drawn) is applied on the frist contour.
       * If using a set-valued friction law, then the arrow is drawn in green if the contact
       * is in slip and in red, if the contact is in stick.
       */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVTangentialForce, tag, (optional (sideOfInteraction,(OpenMBVInteractionArrow::SideOfInteraction),OpenMBVInteractionArrow::action)(scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        ombvFriction = std::shared_ptr<OpenMBVFrictionArrow>(new OpenMBVFrictionArrow(sideOfInteraction,scaleLength,scaleSize,OpenMBV::Arrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency));
      }
  };

}  

#endif
