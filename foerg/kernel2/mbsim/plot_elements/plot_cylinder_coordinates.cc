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

  PlotCylinderCoordinates::PlotCylinderCoordinates(const std::string &name) : Element(name), frame(0), rscale(1), vscale(1), ascale(1) {
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
            openMBVPosition->setName("Position");
            getOpenMBVGrp()->addObject(openMBVPosition);
          }
          if(openMBVVelocity) {
            openMBVVelocity->setName("Velocity");
            getOpenMBVGrp()->addObject(openMBVVelocity);
          }
          if(openMBVRadialVelocity) {
            openMBVRadialVelocity->setName("RadialVelocity");
            getOpenMBVGrp()->addObject(openMBVRadialVelocity);
          }
          if(openMBVCircularVelocity) {
            openMBVCircularVelocity->setName("CircularVelocity");
            getOpenMBVGrp()->addObject(openMBVCircularVelocity);
          }
          if(openMBVZVelocity) {
            openMBVZVelocity->setName("ZVelocity");
            getOpenMBVGrp()->addObject(openMBVZVelocity);
          }
          if(openMBVAcceleration) {
            openMBVAcceleration->setName("Acceleration");
            getOpenMBVGrp()->addObject(openMBVAcceleration);
          }
          if(openMBVRadialAcceleration) {
            openMBVRadialAcceleration->setName("RadialAcceleration");
            getOpenMBVGrp()->addObject(openMBVRadialAcceleration);
          }
          if(openMBVCircularAcceleration) {
            openMBVCircularAcceleration->setName("CircularAcceleration");
            getOpenMBVGrp()->addObject(openMBVCircularAcceleration);
          }
          if(openMBVZAcceleration) {
            openMBVZAcceleration->setName("ZAcceleration");
            getOpenMBVGrp()->addObject(openMBVZAcceleration);
          }
          if(openMBVFrame) {
            openMBVFrame->setName("Frame");
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

  void PlotCylinderCoordinates::enableOpenMBVVelocity(double scale, double diameter, double headDiameter, double headLength, double color) {
    openMBVVelocity=new OpenMBV::Arrow;
    openMBVVelocity->setDiameter(diameter);
    openMBVVelocity->setHeadDiameter(headDiameter);
    openMBVVelocity->setHeadLength(headLength);
    openMBVVelocity->setStaticColor(color);
    openMBVRadialVelocity=new OpenMBV::Arrow;
    openMBVRadialVelocity->setDiameter(diameter);
    openMBVRadialVelocity->setHeadDiameter(headDiameter);
    openMBVRadialVelocity->setHeadLength(headLength);
    openMBVRadialVelocity->setStaticColor(color);
    openMBVCircularVelocity=new OpenMBV::Arrow;
    openMBVCircularVelocity->setDiameter(diameter);
    openMBVCircularVelocity->setHeadDiameter(headDiameter);
    openMBVCircularVelocity->setHeadLength(headLength);
    openMBVCircularVelocity->setStaticColor(color);
    openMBVZVelocity=new OpenMBV::Arrow;
    openMBVZVelocity->setDiameter(diameter);
    openMBVZVelocity->setHeadDiameter(headDiameter);
    openMBVZVelocity->setHeadLength(headLength);
    openMBVZVelocity->setStaticColor(color);
    vscale = scale;
  }

  void PlotCylinderCoordinates::enableOpenMBVAcceleration(double scale, double diameter, double headDiameter, double headLength, double color) {
    openMBVAcceleration=new OpenMBV::Arrow;
    openMBVAcceleration->setDiameter(diameter);
    openMBVAcceleration->setHeadDiameter(headDiameter);
    openMBVAcceleration->setHeadLength(headLength);
    openMBVAcceleration->setStaticColor(color);
    openMBVRadialAcceleration=new OpenMBV::Arrow;
    openMBVRadialAcceleration->setDiameter(diameter);
    openMBVRadialAcceleration->setHeadDiameter(headDiameter);
    openMBVRadialAcceleration->setHeadLength(headLength);
    openMBVRadialAcceleration->setStaticColor(color);
    openMBVCircularAcceleration=new OpenMBV::Arrow;
    openMBVCircularAcceleration->setDiameter(diameter);
    openMBVCircularAcceleration->setHeadDiameter(headDiameter);
    openMBVCircularAcceleration->setHeadLength(headLength);
    openMBVCircularAcceleration->setStaticColor(color);
    openMBVZAcceleration=new OpenMBV::Arrow;
    openMBVZAcceleration->setDiameter(diameter);
    openMBVZAcceleration->setHeadDiameter(headDiameter);
    openMBVZAcceleration->setHeadLength(headLength);
    openMBVZAcceleration->setStaticColor(color);
    ascale = scale;
  }

  void PlotCylinderCoordinates::enableOpenMBVFrame(double size, double offset) {
    openMBVFrame=new OpenMBV::Frame;
    openMBVFrame->setSize(size);
    openMBVFrame->setOffset(offset);
  }
#endif

  void PlotCylinderCoordinates::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        Vec3 r = frame->getPosition();
        Vec3 v = frame->getVelocity();
        Vec3 a = frame->getAcceleration();
        Vec3 ep = crossProduct(ez,r);
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
        Vec3 er = crossProduct(ep,ez);

        v *= vscale;
        a *= ascale;

        if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(0.5);
          openMBVPosition->append(data);
        }

        if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
          vector<double> data;
          Vec3 off = v;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(v(0));
          data.push_back(v(1));
          data.push_back(v(2));
          data.push_back(0.5);
          openMBVVelocity->append(data);
        }

        if(openMBVRadialVelocity && !openMBVRadialVelocity->isHDF5Link()) {
          vector<double> data;
          Vec3 vr =  (v.T()*er)*er;
          Vec3 off = vr;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(vr(0));
          data.push_back(vr(1));
          data.push_back(vr(2));
          data.push_back(0.5);
          openMBVRadialVelocity->append(data);
        }

        if(openMBVCircularVelocity && !openMBVCircularVelocity->isHDF5Link()) {
          vector<double> data;
          Vec3 vp =  (v.T()*ep)*ep;
          Vec3 off = vp;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(vp(0));
          data.push_back(vp(1));
          data.push_back(vp(2));
          data.push_back(0.5);
          openMBVCircularVelocity->append(data);
        }

        if(openMBVZVelocity && !openMBVZVelocity->isHDF5Link()) {
          vector<double> data;
          Vec3 vz =  (v.T()*ez)*ez;
          Vec3 off = vz;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(vz(0));
          data.push_back(vz(1));
          data.push_back(vz(2));
          data.push_back(0.5);
          openMBVZVelocity->append(data);
        }

        if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec3 off = a;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(a(0));
          data.push_back(a(1));
          data.push_back(a(2));
          data.push_back(0.5);
          openMBVAcceleration->append(data);
        }

        if(openMBVRadialAcceleration && !openMBVRadialAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec3 ar =  (a.T()*er)*er;
          Vec3 off = ar;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(ar(0));
          data.push_back(ar(1));
          data.push_back(ar(2));
          data.push_back(0.5);
          openMBVRadialAcceleration->append(data);
        }

        if(openMBVCircularAcceleration && !openMBVCircularAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec3 ap =  (a.T()*ep)*ep;
          Vec3 off = ap;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(ap(0));
          data.push_back(ap(1));
          data.push_back(ap(2));
          data.push_back(0.5);
          openMBVCircularAcceleration->append(data);
        }

        if(openMBVZAcceleration && !openMBVZAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec3 az =  (a.T()*ez)*ez;
          Vec3 off = az;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(az(0));
          data.push_back(az(1));
          data.push_back(az(2));
          data.push_back(0.5);
          openMBVZAcceleration->append(data);
        }

        if(openMBVFrame && !openMBVFrame->isHDF5Link()) {
          vector<double> data;
          SqrMat3 AWP(3);
          AWP.set(0, er);
          AWP.set(1, ep);
          AWP.set(2, ez);
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          Vec3 cardan=AIK2Cardan(AWP);
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
