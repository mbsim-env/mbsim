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

#ifndef _PLOT_FRAME_H__
#define _PLOT_FRAME_H__
#include "mbsim/element.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  //class Frame;
  class Arrow;
  class Frame;
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
      fmatvec::Vec roff, voff, aoff;
      double rscale, vscale, ascale;
#endif

    public:
      PlotFrame(const std::string &name);
      void setFrame(Frame *frame_) { frame = frame_; } 

      void init(InitStage stage);
      virtual void plot(double t, double dt);

#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual void enableOpenMBVPosition(double scale=1, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5, const fmatvec::Vec& off=fmatvec::Vec(3));
      virtual void enableOpenMBVVelocity(double scale=1, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5, const fmatvec::Vec& off=fmatvec::Vec(3));
      virtual void enableOpenMBVAcceleration(double scale=1, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5, const fmatvec::Vec& off=fmatvec::Vec(3));
      virtual void enableOpenMBVAngularVelocity(double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVAngularAcceleration(double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
#endif

  };

}  

#endif

