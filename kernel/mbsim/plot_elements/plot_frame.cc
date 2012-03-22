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

#include <config.h>
#include "mbsim/plot_elements/plot_frame.h"
#include "mbsim/rigid_body.h"
#include "mbsim/frame.h"
#include "mbsim/environment.h"
#include "mbsim/utils/rotarymatrices.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/frame.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  PlotFrame::PlotFrame(const std::string &name) : Element(name), frame(0), roff(3), voff(3), aoff(3), rscale(1), vscale(1), ascale(1) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVPosition=0;
    openMBVVelocity=0;
    openMBVAcceleration=0;
    openMBVAngularVelocity=0;
    openMBVAngularAcceleration=0;
#endif
  }

  void PlotFrame::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          openMBVGrp=new OpenMBV::Group();
          openMBVGrp->setName(name+"_Group");
          openMBVGrp->setExpand(false);
          parent->getOpenMBVGrp()->addObject(openMBVGrp);
          if(openMBVPosition) {
            openMBVPosition->setName(name+"_Position");
            getOpenMBVGrp()->addObject(openMBVPosition);
          }
          if(openMBVVelocity) {
            openMBVVelocity->setName(name+"_Velocity");
            getOpenMBVGrp()->addObject(openMBVVelocity);
          }
          if(openMBVAcceleration) {
            openMBVAcceleration->setName(name+"_Acceleration");
            getOpenMBVGrp()->addObject(openMBVAcceleration);
          }
          if(openMBVAngularVelocity) {
            openMBVAngularVelocity->setName(name+"_AngularVelocity");
            getOpenMBVGrp()->addObject(openMBVAngularVelocity);
          }
          if(openMBVAngularAcceleration) {
            openMBVAngularAcceleration->setName(name+"_AngularAcceleration");
            getOpenMBVGrp()->addObject(openMBVAngularAcceleration);
          }
        }
#endif
      }
      Element::init(stage);
    }
    else
      Element::init(stage);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void PlotFrame::enableOpenMBVPosition(double scale, double diameter, double headDiameter, double headLength, double color, const Vec& off) {
    openMBVPosition=new OpenMBV::Arrow;
    openMBVPosition->setDiameter(diameter);
    openMBVPosition->setHeadDiameter(headDiameter);
    openMBVPosition->setHeadLength(headLength);
    openMBVPosition->setStaticColor(color);
    roff = off;
    rscale = scale;
  }

  void PlotFrame::enableOpenMBVVelocity(double scale, double diameter, double headDiameter, double headLength, double color, const Vec& off) {
    openMBVVelocity=new OpenMBV::Arrow;
    openMBVVelocity->setDiameter(diameter);
    openMBVVelocity->setHeadDiameter(headDiameter);
    openMBVVelocity->setHeadLength(headLength);
    openMBVVelocity->setStaticColor(color);
    voff = off;
    vscale = scale;
  }

  void PlotFrame::enableOpenMBVAcceleration(double scale, double diameter, double headDiameter, double headLength, double color, const Vec& off) {
    openMBVAcceleration=new OpenMBV::Arrow;
    openMBVAcceleration->setDiameter(diameter);
    openMBVAcceleration->setHeadDiameter(headDiameter);
    openMBVAcceleration->setHeadLength(headLength);
    openMBVAcceleration->setStaticColor(color);
    aoff = off;
    ascale = scale;
  }

  void PlotFrame::enableOpenMBVAngularVelocity(double diameter, double headDiameter, double headLength, double color) {
    openMBVAngularVelocity=new OpenMBV::Arrow;
    openMBVAngularVelocity->setDiameter(diameter);
    openMBVAngularVelocity->setHeadDiameter(headDiameter);
    openMBVAngularVelocity->setHeadLength(headLength);
    openMBVAngularVelocity->setStaticColor(color);
  }

  void PlotFrame::enableOpenMBVAngularAcceleration(double diameter, double headDiameter, double headLength, double color) {
    openMBVAngularAcceleration=new OpenMBV::Arrow;
    openMBVAngularAcceleration->setDiameter(diameter);
    openMBVAngularAcceleration->setHeadDiameter(headDiameter);
    openMBVAngularAcceleration->setHeadLength(headLength);
    openMBVAngularAcceleration->setStaticColor(color);
  }
#endif

  void PlotFrame::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0)+roff(0));
          data.push_back(frame->getPosition()(1)+roff(1));
          data.push_back(frame->getPosition()(2)+roff(2));
          data.push_back(frame->getPosition()(0)+roff(0));
          data.push_back(frame->getPosition()(1)+roff(1));
          data.push_back(frame->getPosition()(2)+roff(2));
          data.push_back(0.5);
          openMBVPosition->append(data);
        }
        if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          Vec vframe = frame->getVelocity()*vscale;
          Vec off = voff + vframe;
          data.push_back(frame->getPosition()(0)+off(0));
          data.push_back(frame->getPosition()(1)+off(1));
          data.push_back(frame->getPosition()(2)+off(2));
          data.push_back(vframe(0));
          data.push_back(vframe(1));
          data.push_back(vframe(2));
          data.push_back(0.5);
          openMBVVelocity->append(data);
        }
        if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          Vec aframe = frame->getAcceleration()*ascale;
          Vec off = aoff + aframe;
          data.push_back(frame->getPosition()(0)+off(0));
          data.push_back(frame->getPosition()(1)+off(1));
          data.push_back(frame->getPosition()(2)+off(2));
          data.push_back(aframe(0));
          data.push_back(aframe(1));
          data.push_back(aframe(2));
          data.push_back(0.5);
          openMBVAcceleration->append(data);
        }
        if(openMBVAngularVelocity && !openMBVAngularVelocity->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getAngularVelocity()(0));
          data.push_back(frame->getAngularVelocity()(1));
          data.push_back(frame->getAngularVelocity()(2));
          data.push_back(0.5);
          openMBVAngularVelocity->append(data);
        }
        if(openMBVAngularAcceleration && !openMBVAngularAcceleration->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getAngularAcceleration()(0));
          data.push_back(frame->getAngularAcceleration()(1));
          data.push_back(frame->getAngularAcceleration()(2));
          data.push_back(0.5);
          openMBVAngularAcceleration->append(data);
        }
      }
#endif

      Element::plot(t,dt);
    }
  }
}
