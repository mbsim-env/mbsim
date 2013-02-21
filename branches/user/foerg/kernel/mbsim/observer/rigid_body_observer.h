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

#ifndef _RIGID_BODY_OBSERVER_H__
#define _RIGID_BODY_OBSERVER_H__
#include "mbsim/observer/observer.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/arrow.h>
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
      OpenMBV::Group* openMBVGrp;
      OpenMBV::Arrow *openMBVAxisOfRotation;
#endif

    public:
      RigidBodyObserver(const std::string &name);
      void setRigidBody(RigidBody *body_) { body = body_; } 

      void init(InitStage stage);
      virtual void plot(double t, double dt);

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBVAxisOfRotation(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
#endif

  };

}  

#endif

