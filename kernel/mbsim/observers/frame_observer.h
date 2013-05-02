/* Copyright (C) 2004-2013 MBSim Development Team
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

#ifndef _FRAME_OBSERVER_H__
#define _FRAME_OBSERVER_H__
#include "mbsim/observer.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/arrow.h>
#endif

namespace MBSim {
  class Frame;

  class FrameObserver : public Observer {
    private:
      Frame* frame;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *openMBVPositionArrow, *openMBVVelocityArrow, *openMBVAngularVelocityArrow, *openMBVAccelerationArrow, *openMBVAngularAccelerationArrow;
#endif

    public:
      FrameObserver(const std::string &name);
      void setFrame(Frame *frame_) { frame = frame_; } 

      void init(InitStage stage);
      virtual void plot(double t, double dt);

#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVPositionArrow(OpenMBV::Arrow *arrow) { openMBVPositionArrow = arrow; }
      void setOpenMBVVelocityArrow(OpenMBV::Arrow *arrow) { openMBVVelocityArrow = arrow; }
      void setOpenMBVAngularVelocityArrow(OpenMBV::Arrow *arrow) { openMBVAngularVelocityArrow = arrow; }
      void setOpenMBVAccelerationArrow(OpenMBV::Arrow *arrow) { openMBVAccelerationArrow = arrow; }
      void setOpenMBVAngularAccelerationArrow(OpenMBV::Arrow *arrow) { openMBVAngularAccelerationArrow = arrow; }

      virtual void enableOpenMBVPosition(double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVVelocity(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVAngularVelocity(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVAcceleration(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVAngularAcceleration(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
#endif

  };

}  

#endif

