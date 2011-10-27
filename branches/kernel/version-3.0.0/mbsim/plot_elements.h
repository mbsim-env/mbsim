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

#ifndef _PLOT_ELEMENTS_H__
#define _PLOT_ELEMENTS_H__
#include "mbsim/element.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  //class Frame;
  class Arrow;
}
#endif

namespace MBSim {
  class RigidBody;
  class Frame;

  class PlotFrame : public Element {
    private:
      Frame* frame;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *openMBVPosition, *openMBVVelocity, *openMBVAcceleration, *openMBVAngularVelocity, *openMBVAngularAcceleration;
#endif

    public:
      PlotFrame(const std::string &name);
      void setFrame(Frame *frame_) { frame = frame_; } 

      void init(InitStage stage);
      virtual void plot(double t, double dt);

#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual void enableOpenMBVPosition(double scaleLength=1, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVVelocity(double scaleLength=1, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVAcceleration(double scaleLength=1, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVAngularVelocity(double scaleLength=1, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVAngularAcceleration(double scaleLength=1, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
#endif

  };

  class PlotRigidBodyGroup : public Element {
    private:
      std::vector<RigidBody*> body;
      Frame* ref;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *openMBVPosition, *openMBVVelocity, *openMBVAcceleration, *openMBVAngularVelocity, *openMBVAngularAcceleration, *openMBVWeight, *openMBVMomentum, *openMBVAngularMomentum, *openMBVDerivativeOfMomentum, *openMBVDerivativeOfAngularMomentum;
#endif

    public:
      PlotRigidBodyGroup(const std::string &name);
      void addBody(RigidBody *body_) { body.push_back(body_); } 
      void setFrameOfReference(Frame* frame) { ref = frame; } 

      void init(InitStage stage);
      virtual void plot(double t, double dt);

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBVWeight(double scaleLength, double diameter, double headDiameter, double headLength, double color=0.5); 
      void enableOpenMBVMomentum(double scaleLength, double diameter, double headDiameter, double headLength, double color=0.5); 
      void enableOpenMBVAngularMomentum(double scaleLength, double diameter, double headDiameter, double headLength, double color=0.5);
      void enableOpenMBVDerivativeOfMomentum(double scaleLength, double diameter, double headDiameter, double headLength, double color=0.5);
      void enableOpenMBVDerivativeOfAngularMomentum(double scaleLength, double diameter, double headDiameter, double headLength, double color=0.5);
#endif

  };

}

#endif

