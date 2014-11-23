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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _RIGID_BODY_GROUP_OBSERVER_H__
#define _RIGID_BODY_GROUP_OBSERVER_H__
#include "mbsim/observer.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>

namespace OpenMBV {
  class Frame;
}
#endif

namespace MBSim {
  class RigidBody;
  class Frame;

  class RigidBodyGroupObserver : public Observer {
    private:
      std::vector<RigidBody*> body;
      Frame* ref;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *openMBVPosition, *openMBVVelocity, *openMBVAcceleration, *openMBVAngularVelocity, *openMBVAngularAcceleration, *openMBVWeight, *openMBVMomentum, *openMBVAngularMomentum, *openMBVDerivativeOfMomentum, *openMBVDerivativeOfAngularMomentum;
#endif

    public:
      RigidBodyGroupObserver(const std::string &name);
      void addBody(RigidBody *body_) { body.push_back(body_); } 
      void setFrameOfReference(Frame* frame) { ref = frame; } 

      void init(InitStage stage);
      virtual void plot(double t, double dt);

#ifdef HAVE_OPENMBVCPPINTERFACE
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVWeight, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        openMBVWeight=ombv.createOpenMBV(); 
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMomentum, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        openMBVMomentum=ombv.createOpenMBV(); 
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAngularMomentum, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toDoubleHead,referencePoint,scaleLength,scaleSize);
        openMBVAngularMomentum=ombv.createOpenMBV(); 
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVDerivativeOfMomentum, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        openMBVDerivativeOfMomentum=ombv.createOpenMBV(); 
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVDerivativeOfAngularMomentum, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toDoubleHead,referencePoint,scaleLength,scaleSize);
        openMBVDerivativeOfAngularMomentum=ombv.createOpenMBV(); 
      }
#endif

  };

}  

#endif

