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

#ifndef _SINGLE_CONTACT_OBSERVER_H__
#define _SINGLE_CONTACT_OBSERVER_H__

#include "mbsim/observers/mechanical_link_observer.h"

#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  class SingleContact;

  class SingleContactObserver : public MechanicalLinkObserver {

    friend class ContactObserver;
    friend class MaxwellContactObserver;

    protected:
      /**
       * \brief container of ContactFrames to draw
       */
      std::vector<std::shared_ptr<OpenMBV::Frame> > openMBVContactFrame;

      /**
       * \brief pointer to memory of normal and friction forces to draw
       */
      std::shared_ptr<OpenMBV::Arrow> contactArrow, frictionArrow;

    public:
      SingleContactObserver(const std::string &name="");

      void init(InitStage stage, const InitConfigSet &config) override;
      void plot() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      /** 
       * \brief Draw two OpenMBV::Frame's of size 'size' at the contact points if 'enable'==true, otherwise the object is available but disabled.
       * If the contact is closed, then the two contact points are the same on each contour.
       * If the contact is not closed, then the two contact point lie on the contours with minimal distance in between.
       * The x-axis of this frames are orientated to the other frame origin (normal vector).
       */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVContactPoints, tag, (optional (size,(double),1)(offset,(double),1)(transparency,(double),0))) {
        OpenMBVFrame ombv(size,offset,"[-1;1;1]",transparency);
        setOpenMBVContactPoints(ombv.createOpenMBV());
      }

      void setOpenMBVContactPoints(const std::shared_ptr<OpenMBV::Frame> &frame) { 
        openMBVContactFrame[0]=frame;
        openMBVContactFrame[1]=OpenMBV::ObjectFactory::create(openMBVContactFrame[0]);
      }

      /** 
       * \brief Sets the OpenMBV::Arrow to be used for drawing the normal force vector.
       * This vector is the force which is applied on the second contour.
       * The reactio (not drawn) is applied on the first contour.
       */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVNormalForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        contactArrow=ombv.createOpenMBV(); 
      }

      void setOpenMBVNormalForce(const std::shared_ptr<OpenMBV::Arrow> &arrow) { contactArrow=arrow; }

      /** 
       * \brief Sets the OpenMBV::Arrow to be used for drawing the friction force vector.
       * This vector is the friction which is applied on the second contour.
       * The reactio (not drawn) is applied on the frist contour.
       * If using a set-valued friction law, then the arrow is drawn in green if the contact
       * is in slip and in red, if the contact is in stick.
       */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVTangentialForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        frictionArrow=ombv.createOpenMBV(); 
      }

      void setOpenMBVTangentialForce(const std::shared_ptr<OpenMBV::Arrow> &arrow) { frictionArrow=arrow; }
  };

}  

#endif
