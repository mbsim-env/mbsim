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
#include "mbsim/plot_elements/plot_cylinder_coordinates.h"
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

  PlotCylinderCoordinates::PlotCylinderCoordinates(const std::string &name) : Element(name), frame(0), roff(3), voff(3), aoff(3), rscale(1), vscale(1), ascale(1), ez(3) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVPosition=0;
    openMBVVelocity=0;
    openMBVRadialVelocity=0;
    openMBVCircularVelocity=0;
    openMBVZVelocity=0;
    openMBVAcceleration=0;
    openMBVRadialAcceleration=0;
    openMBVCircularAcceleration=0;
    openMBVZAcceleration=0;
    openMBVFrame=0;
#endif
    ez(2) = 1;
  }

  void PlotCylinderCoordinates::init(InitStage stage) {
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
          if(openMBVRadialVelocity) {
            openMBVRadialVelocity->setName(name+"_RadialVelocity");
            getOpenMBVGrp()->addObject(openMBVRadialVelocity);
          }
          if(openMBVCircularVelocity) {
            openMBVCircularVelocity->setName(name+"_CircularVelocity");
            getOpenMBVGrp()->addObject(openMBVCircularVelocity);
          }
          if(openMBVZVelocity) {
            openMBVZVelocity->setName(name+"_ZVelocity");
            getOpenMBVGrp()->addObject(openMBVZVelocity);
          }
          if(openMBVAcceleration) {
            openMBVAcceleration->setName(name+"_Acceleration");
            getOpenMBVGrp()->addObject(openMBVAcceleration);
          }
          if(openMBVRadialAcceleration) {
            openMBVRadialAcceleration->setName(name+"_RadialAcceleration");
            getOpenMBVGrp()->addObject(openMBVRadialAcceleration);
          }
          if(openMBVCircularAcceleration) {
            openMBVCircularAcceleration->setName(name+"_CircularAcceleration");
            getOpenMBVGrp()->addObject(openMBVCircularAcceleration);
          }
          if(openMBVZAcceleration) {
            openMBVZAcceleration->setName(name+"_ZAcceleration");
            getOpenMBVGrp()->addObject(openMBVZAcceleration);
          }
          if(openMBVFrame) {
            openMBVFrame->setName(name+"_Dreibein");
            getOpenMBVGrp()->addObject(openMBVFrame);
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
  void PlotCylinderCoordinates::enableOpenMBVPosition(double diameter, double headDiameter, double headLength, double color) {
    openMBVPosition=new OpenMBV::Arrow;
    openMBVPosition->setDiameter(diameter);
    openMBVPosition->setHeadDiameter(headDiameter);
    openMBVPosition->setHeadLength(headLength);
    openMBVPosition->setStaticColor(color);
  }

  void PlotCylinderCoordinates::enableOpenMBVVelocity(double diameter, double headDiameter, double headLength, double color) {
    openMBVVelocity=new OpenMBV::Arrow;
    openMBVVelocity->setDiameter(diameter);
    openMBVVelocity->setHeadDiameter(headDiameter);
    openMBVVelocity->setHeadLength(headLength);
    openMBVVelocity->setStaticColor(color);
  }

  void PlotCylinderCoordinates::enableOpenMBVRadialVelocity(double diameter, double headDiameter, double headLength, double color) {
    openMBVRadialVelocity=new OpenMBV::Arrow;
    openMBVRadialVelocity->setDiameter(diameter);
    openMBVRadialVelocity->setHeadDiameter(headDiameter);
    openMBVRadialVelocity->setHeadLength(headLength);
    openMBVRadialVelocity->setStaticColor(color);
  }

  void PlotCylinderCoordinates::enableOpenMBVCircularVelocity(double diameter, double headDiameter, double headLength, double color) {
    openMBVCircularVelocity=new OpenMBV::Arrow;
    openMBVCircularVelocity->setDiameter(diameter);
    openMBVCircularVelocity->setHeadDiameter(headDiameter);
    openMBVCircularVelocity->setHeadLength(headLength);
    openMBVCircularVelocity->setStaticColor(color);
  }

  void PlotCylinderCoordinates::enableOpenMBVZVelocity(double diameter, double headDiameter, double headLength, double color) {
    openMBVZVelocity=new OpenMBV::Arrow;
    openMBVZVelocity->setDiameter(diameter);
    openMBVZVelocity->setHeadDiameter(headDiameter);
    openMBVZVelocity->setHeadLength(headLength);
    openMBVZVelocity->setStaticColor(color);
  }

  void PlotCylinderCoordinates::enableOpenMBVAcceleration(double diameter, double headDiameter, double headLength, double color) {
    openMBVAcceleration=new OpenMBV::Arrow;
    openMBVAcceleration->setDiameter(diameter);
    openMBVAcceleration->setHeadDiameter(headDiameter);
    openMBVAcceleration->setHeadLength(headLength);
    openMBVAcceleration->setStaticColor(color);
  }
  void PlotCylinderCoordinates::enableOpenMBVRadialAcceleration(double diameter, double headDiameter, double headLength, double color) {
    openMBVRadialAcceleration=new OpenMBV::Arrow;
    openMBVRadialAcceleration->setDiameter(diameter);
    openMBVRadialAcceleration->setHeadDiameter(headDiameter);
    openMBVRadialAcceleration->setHeadLength(headLength);
    openMBVRadialAcceleration->setStaticColor(color);
  }
  void PlotCylinderCoordinates::enableOpenMBVCircularAcceleration(double diameter, double headDiameter, double headLength, double color) {
    openMBVCircularAcceleration=new OpenMBV::Arrow;
    openMBVCircularAcceleration->setDiameter(diameter);
    openMBVCircularAcceleration->setHeadDiameter(headDiameter);
    openMBVCircularAcceleration->setHeadLength(headLength);
    openMBVCircularAcceleration->setStaticColor(color);
  }
  void PlotCylinderCoordinates::enableOpenMBVZAcceleration(double diameter, double headDiameter, double headLength, double color) {
    openMBVZAcceleration=new OpenMBV::Arrow;
    openMBVZAcceleration->setDiameter(diameter);
    openMBVZAcceleration->setHeadDiameter(headDiameter);
    openMBVZAcceleration->setHeadLength(headLength);
    openMBVZAcceleration->setStaticColor(color);
  }
  void PlotCylinderCoordinates::enableOpenMBVFrame(double diameter, double headDiameter, double headLength, double color) {
    openMBVFrame=new OpenMBV::Frame;
    openMBVFrame->setSize(1);
    openMBVFrame->setOffset(1);
  }
#endif

  void PlotCylinderCoordinates::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        Vec r = frame->getPosition();
        Vec v = frame->getVelocity()*vscale;
        Vec a = frame->getAcceleration()*ascale;
        Vec ep = crossProduct(ez,r);
        double nrmep = nrm2(ep);
        if(nrmep<epsroot()) {
          if(fabs(ez(0))<epsroot() && fabs(ez(1))<epsroot()) {
            ep(0) = 1.;
            ep(1) = 0.;
            ep(2) = 0.;
          }
          else {
            ep(0) = -ez(1);
            ep(1) = ez(0);
            ep(2) = 0.0;
          }
        }
        else
          ep = ep/nrmep;
        Vec er = crossProduct(ep,ez);
        if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(0.5);
          openMBVPosition->append(data);
        }
        if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
          vector<double> data;
          Vec off = v;
          data.push_back(t);
          data.push_back(frame->getPosition()(0)+off(0));
          data.push_back(frame->getPosition()(1)+off(1));
          data.push_back(frame->getPosition()(2)+off(2));
          data.push_back(v(0));
          data.push_back(v(1));
          data.push_back(v(2));
          data.push_back(0.5);
          openMBVVelocity->append(data);
        }
        if(openMBVRadialVelocity && !openMBVRadialVelocity->isHDF5Link()) {
          vector<double> data;
          Vec vr =  (v.T()*er)*er;
          Vec off = vr;
          data.push_back(t);
          data.push_back(frame->getPosition()(0)+off(0));
          data.push_back(frame->getPosition()(1)+off(1));
          data.push_back(frame->getPosition()(2)+off(2));
          data.push_back(vr(0));
          data.push_back(vr(1));
          data.push_back(vr(2));
          data.push_back(0.5);
          openMBVRadialVelocity->append(data);
        }
        if(openMBVCircularVelocity && !openMBVCircularVelocity->isHDF5Link()) {
          vector<double> data;
          Vec vp =  (v.T()*ep)*ep;
          //Vec vz =  (v.T()*ez)*ez;
          Vec off = vp;
          data.push_back(t);
          data.push_back(frame->getPosition()(0)+off(0));
          data.push_back(frame->getPosition()(1)+off(1));
          data.push_back(frame->getPosition()(2)+off(2));
          data.push_back(vp(0));
          data.push_back(vp(1));
          data.push_back(vp(2));
          data.push_back(0.5);
          openMBVCircularVelocity->append(data);
        }
        if(openMBVZVelocity && !openMBVZVelocity->isHDF5Link()) {
          vector<double> data;
          Vec vz =  (v.T()*ez)*ez;
          Vec off = vz;
          data.push_back(t);
          data.push_back(frame->getPosition()(0)+off(0));
          data.push_back(frame->getPosition()(1)+off(1));
          data.push_back(frame->getPosition()(2)+off(2));
          data.push_back(vz(0));
          data.push_back(vz(1));
          data.push_back(vz(2));
          data.push_back(0.5);
          openMBVZVelocity->append(data);
        }
        if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec off = a;
          data.push_back(t);
          data.push_back(frame->getPosition()(0)+off(0));
          data.push_back(frame->getPosition()(1)+off(1));
          data.push_back(frame->getPosition()(2)+off(2));
          data.push_back(a(0));
          data.push_back(a(1));
          data.push_back(a(2));
          data.push_back(0.5);
          openMBVAcceleration->append(data);
        }
        if(openMBVRadialAcceleration && !openMBVRadialAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec ar =  (a.T()*er)*er;
          Vec off = ar;
          data.push_back(t);
          data.push_back(frame->getPosition()(0)+off(0));
          data.push_back(frame->getPosition()(1)+off(1));
          data.push_back(frame->getPosition()(2)+off(2));
          data.push_back(ar(0));
          data.push_back(ar(1));
          data.push_back(ar(2));
          data.push_back(0.5);
          openMBVRadialAcceleration->append(data);
        }
        if(openMBVCircularAcceleration && !openMBVCircularAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec ap =  (a.T()*ep)*ep;
          Vec off = ap;
          data.push_back(t);
          data.push_back(frame->getPosition()(0)+off(0));
          data.push_back(frame->getPosition()(1)+off(1));
          data.push_back(frame->getPosition()(2)+off(2));
          data.push_back(ap(0));
          data.push_back(ap(1));
          data.push_back(ap(2));
          data.push_back(0.5);
          openMBVCircularAcceleration->append(data);
        }
        if(openMBVZAcceleration && !openMBVZAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec az =  (a.T()*ez)*ez;
          Vec off = az;
          data.push_back(t);
          data.push_back(frame->getPosition()(0)+off(0));
          data.push_back(frame->getPosition()(1)+off(1));
          data.push_back(frame->getPosition()(2)+off(2));
          data.push_back(az(0));
          data.push_back(az(1));
          data.push_back(az(2));
          data.push_back(0.5);
          openMBVZAcceleration->append(data);
        }
        if(openMBVFrame && !openMBVFrame->isHDF5Link()) {
          vector<double> data;
          SqrMat AWP(3);
          AWP.col(0) = er;
          AWP.col(1) = ep;
          AWP.col(2) = ez;
          vector<double> data3;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          Vec cardan=AIK2Cardan(AWP);
          data.push_back(cardan(0));
          data.push_back(cardan(1));
          data.push_back(cardan(2));
          data.push_back(0);
          openMBVFrame->append(data);

        }
      }
#endif

      Element::plot(t,dt);
    }
  }


}
