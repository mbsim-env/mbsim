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

#ifndef _CARTESIAN_COORDINATES_OBSERVER_H__
#define _CARTESIAN_COORDINATES_OBSERVER_H__
#include "mbsim/observer.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/arrow.h>

namespace OpenMBV {
  class Frame;
}
#endif

namespace MBSim {
  class Frame;

  class CartesianCoordinatesObserver : public Observer {
    private:
      Frame* frame;
      fmatvec::Vec3 ex, ey, ez;
      fmatvec::SqrMat3 A;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *openMBVPosition,*openMBVXPosition, *openMBVYPosition, *openMBVZPosition, *openMBVVelocity, *openMBVXVelocity, *openMBVYVelocity, *openMBVZVelocity, *openMBVAcceleration, *openMBVXAcceleration, *openMBVYAcceleration, *openMBVZAcceleration; 
      OpenMBV::Frame* openMBVFrame;
#endif

    public:
      CartesianCoordinatesObserver(const std::string &name);
      void setFrame(Frame *frame_) { frame = frame_; } 
      void setOrientation(const fmatvec::SqrMat3 &A_) { A = A_; } 

      void init(InitStage stage);
      virtual void plot(double t, double dt);

#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual void enableOpenMBVPosition(double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVVelocity(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVAcceleration(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVFrame(double size=1, double offset=1);
#endif

  };

}  

#endif

