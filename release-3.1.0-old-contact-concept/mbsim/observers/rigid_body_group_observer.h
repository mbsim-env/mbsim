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
#include <openmbvcppinterface/arrow.h>
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
      void enableOpenMBVWeight(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      void enableOpenMBVMomentum(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      void enableOpenMBVAngularMomentum(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      void enableOpenMBVDerivativeOfMomentum(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      void enableOpenMBVDerivativeOfAngularMomentum(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
#endif

  };

}  

#endif

