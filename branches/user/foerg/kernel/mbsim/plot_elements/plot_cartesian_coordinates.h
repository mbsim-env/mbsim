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

#ifndef _PLOT_CYLINDER_COORDINATES_H__
#define _PLOT_CYLINDER_COORDINATES_H__
#include "mbsim/element.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  //class Frame;
  class Arrow;
  class Frame;
  class Group;
}
#endif

namespace MBSim {
  class RigidBody;
  class Frame;

  class PlotCartesianCoordinates : public Element {
    private:
      Frame* frame;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group* openMBVGrp;
      OpenMBV::Arrow *openMBVPosition, *openMBVVelocity, *openMBVXVelocity, *openMBVYVelocity, *openMBVZVelocity, *openMBVAcceleration, *openMBVXAcceleration, *openMBVYAcceleration, *openMBVZAcceleration; 
      OpenMBV::Frame* openMBVFrame;
      fmatvec::Vec3 roff, voff, aoff;
      double rscale, vscale, ascale;
      fmatvec::Vec3 ex, ey, ez;
#endif

    public:
      PlotCartesianCoordinates(const std::string &name);
      void setFrame(Frame *frame_) { frame = frame_; } 

      void init(InitStage stage);
      void setez(const fmatvec::Vec &ez_) {ez = ez_/nrm2(ez_);}
      virtual void plot(double t, double dt);

#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group* getOpenMBVGrp() { return openMBVGrp; }
      //void setvscale(double scale) { vscale = scale; }
      //void setascale(double scale) { ascale = scale; }
      virtual void enableOpenMBVPosition(double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVVelocity(double scale=1, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVAcceleration(double scale=1, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVFrame(double size=1, double offset=1);
#endif

  };

}  

#endif

