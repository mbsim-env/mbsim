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

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <mbsim/utils/openmbv_utils.h>
namespace OpenMBV {
  class Frame;
}
#endif

namespace MBSim {
  class RigidBody;
  class Frame;

  class RigidBodyObserver: public Observer {
    private:
      RigidBody* body;
#ifdef HAVE_OPENMBVCPPINTERFACE
      std::shared_ptr<OpenMBV::Group> openMBVGrp;
      std::shared_ptr<OpenMBV::Arrow> openMBVAxisOfRotation;
#endif

    public:
      RigidBodyObserver(const std::string &name);
      void setRigidBody(RigidBody *body_) { body = body_; } 

      void init(InitStage stage);
      virtual void plot();

#ifdef HAVE_OPENMBVCPPINTERFACE
//      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAxisOfRotation, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(scaleLength,(double),1)(scaleSize,(double),1))) { openMBVAxisOfRotation=enableOpenMBVArrow(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize); }
#endif

  };

}  

#endif

