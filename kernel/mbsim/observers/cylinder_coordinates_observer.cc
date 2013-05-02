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
#include "mbsim/observers/cylinder_coordinates_observer.h"
#include "mbsim/frame.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/utils/eps.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frame.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  CylinderCoordinatesObserver::CylinderCoordinatesObserver(const std::string &name) : Observer(name), frame(0) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVPosition=0;
    openMBVRadialPosition=0;
    openMBVZPosition=0;
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

  void CylinderCoordinatesObserver::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();

      Observer::init(stage);
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVPosition) {
            OpenMBV::Group *openMBVGrp=new OpenMBV::Group();
            openMBVGrp->setName("Position_Group");
            openMBVGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVGrp);
            openMBVPosition->setName("Position");
            openMBVGrp->addObject(openMBVPosition);
            openMBVRadialPosition->setName("RadialPosition");
            openMBVGrp->addObject(openMBVRadialPosition);
            //openMBVYPosition->setName("YPosition");
            //openMBVGrp->addObject(openMBVYPosition);
            openMBVZPosition->setName("ZPosition");
            openMBVGrp->addObject(openMBVZPosition);
          }
          if(openMBVVelocity) {
            OpenMBV::Group *openMBVGrp=new OpenMBV::Group();
            openMBVGrp->setName("Velocity_Group");
            openMBVGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVGrp);
            openMBVVelocity->setName("Velocity");
            openMBVGrp->addObject(openMBVVelocity);
            openMBVRadialVelocity->setName("RadialVelocity");
            openMBVGrp->addObject(openMBVRadialVelocity);
            openMBVCircularVelocity->setName("CircularVelocity");
            openMBVGrp->addObject(openMBVCircularVelocity);
            openMBVZVelocity->setName("ZVelocity");
            openMBVGrp->addObject(openMBVZVelocity);
          }
          if(openMBVAcceleration) {
            OpenMBV::Group *openMBVGrp=new OpenMBV::Group();
            openMBVGrp->setName("Acceleration_Group");
            openMBVGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVGrp);
            openMBVAcceleration->setName("Acceleration");
            openMBVGrp->addObject(openMBVAcceleration);
            openMBVRadialAcceleration->setName("RadialAcceleration");
            openMBVGrp->addObject(openMBVRadialAcceleration);
            openMBVCircularAcceleration->setName("CircularAcceleration");
            openMBVGrp->addObject(openMBVCircularAcceleration);
            openMBVZAcceleration->setName("ZAcceleration");
            openMBVGrp->addObject(openMBVZAcceleration);
          }
          if(openMBVFrame) {
            openMBVFrame->setName("Frame");
            getOpenMBVGrp()->addObject(openMBVFrame);
          }
        }
#endif
      }
    }
    else
      Observer::init(stage);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void CylinderCoordinatesObserver::enableOpenMBVPosition(double diameter, double headDiameter, double headLength, double color) {
    openMBVPosition=new OpenMBV::Arrow;
    openMBVPosition->setReferencePoint(OpenMBV::Arrow::fromPoint);
    openMBVPosition->setDiameter(diameter);
    openMBVPosition->setHeadDiameter(headDiameter);
    openMBVPosition->setHeadLength(headLength);
    openMBVPosition->setStaticColor(color);
    openMBVRadialPosition=new OpenMBV::Arrow;
    openMBVRadialPosition->setReferencePoint(OpenMBV::Arrow::fromPoint);
    openMBVRadialPosition->setDiameter(diameter);
    openMBVRadialPosition->setHeadDiameter(headDiameter);
    openMBVRadialPosition->setHeadLength(headLength);
    openMBVRadialPosition->setStaticColor(color);
    openMBVZPosition=new OpenMBV::Arrow;
    openMBVZPosition->setReferencePoint(OpenMBV::Arrow::fromPoint);
    openMBVZPosition->setDiameter(diameter);
    openMBVZPosition->setHeadDiameter(headDiameter);
    openMBVZPosition->setHeadLength(headLength);
    openMBVZPosition->setStaticColor(color);
  }

  void CylinderCoordinatesObserver::enableOpenMBVVelocity(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVVelocity=new OpenMBV::Arrow;
    openMBVVelocity->setScaleLength(scale);
    openMBVVelocity->setReferencePoint(refPoint);
    openMBVVelocity->setDiameter(diameter);
    openMBVVelocity->setHeadDiameter(headDiameter);
    openMBVVelocity->setHeadLength(headLength);
    openMBVVelocity->setStaticColor(color);
    openMBVRadialVelocity=new OpenMBV::Arrow;
    openMBVRadialVelocity->setScaleLength(scale);
    openMBVRadialVelocity->setReferencePoint(refPoint);
    openMBVRadialVelocity->setDiameter(diameter);
    openMBVRadialVelocity->setHeadDiameter(headDiameter);
    openMBVRadialVelocity->setHeadLength(headLength);
    openMBVRadialVelocity->setStaticColor(color);
    openMBVCircularVelocity=new OpenMBV::Arrow;
    openMBVCircularVelocity->setScaleLength(scale);
    openMBVCircularVelocity->setReferencePoint(refPoint);
    openMBVCircularVelocity->setDiameter(diameter);
    openMBVCircularVelocity->setHeadDiameter(headDiameter);
    openMBVCircularVelocity->setHeadLength(headLength);
    openMBVCircularVelocity->setStaticColor(color);
    openMBVZVelocity=new OpenMBV::Arrow;
    openMBVZVelocity->setScaleLength(scale);
    openMBVZVelocity->setReferencePoint(refPoint);
    openMBVZVelocity->setDiameter(diameter);
    openMBVZVelocity->setHeadDiameter(headDiameter);
    openMBVZVelocity->setHeadLength(headLength);
    openMBVZVelocity->setStaticColor(color);
  }

  void CylinderCoordinatesObserver::enableOpenMBVAcceleration(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVAcceleration=new OpenMBV::Arrow;
    openMBVAcceleration->setScaleLength(scale);
    openMBVAcceleration->setReferencePoint(refPoint);
    openMBVAcceleration->setDiameter(diameter);
    openMBVAcceleration->setHeadDiameter(headDiameter);
    openMBVAcceleration->setHeadLength(headLength);
    openMBVAcceleration->setStaticColor(color);
    openMBVRadialAcceleration=new OpenMBV::Arrow;
    openMBVRadialAcceleration->setScaleLength(scale);
    openMBVRadialAcceleration->setReferencePoint(refPoint);
    openMBVRadialAcceleration->setDiameter(diameter);
    openMBVRadialAcceleration->setHeadDiameter(headDiameter);
    openMBVRadialAcceleration->setHeadLength(headLength);
    openMBVRadialAcceleration->setStaticColor(color);
    openMBVCircularAcceleration=new OpenMBV::Arrow;
    openMBVCircularAcceleration->setScaleLength(scale);
    openMBVCircularAcceleration->setReferencePoint(refPoint);
    openMBVCircularAcceleration->setDiameter(diameter);
    openMBVCircularAcceleration->setHeadDiameter(headDiameter);
    openMBVCircularAcceleration->setHeadLength(headLength);
    openMBVCircularAcceleration->setStaticColor(color);
    openMBVZAcceleration=new OpenMBV::Arrow;
    openMBVZAcceleration->setScaleLength(scale);
    openMBVZAcceleration->setReferencePoint(refPoint);
    openMBVZAcceleration->setDiameter(diameter);
    openMBVZAcceleration->setHeadDiameter(headDiameter);
    openMBVZAcceleration->setHeadLength(headLength);
    openMBVZAcceleration->setStaticColor(color);
  }

  void CylinderCoordinatesObserver::enableOpenMBVFrame(double size, double offset) {
    openMBVFrame=new OpenMBV::Frame;
    openMBVFrame->setSize(size);
    openMBVFrame->setOffset(offset);
  }
#endif

  void CylinderCoordinatesObserver::plot(double t, double dt) {
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

        if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(0.5);
          openMBVPosition->append(data);
          data.clear();
          Vec3 rr =  (r.T()*er)*er;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(rr(0));
          data.push_back(rr(1));
          data.push_back(rr(2));
          data.push_back(0.5);
          openMBVRadialPosition->append(data);
          data.clear();
          Vec3 rz =  (r.T()*ez)*ez;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(rz(0));
          data.push_back(rz(1));
          data.push_back(rz(2));
          data.push_back(0.5);
          openMBVZPosition->append(data);
        }

        if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(v(0));
          data.push_back(v(1));
          data.push_back(v(2));
          data.push_back(0.5);
          openMBVVelocity->append(data);
          data.clear();
          Vec3 vr =  (v.T()*er)*er;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(vr(0));
          data.push_back(vr(1));
          data.push_back(vr(2));
          data.push_back(0.5);
          openMBVRadialVelocity->append(data);
          data.clear();
          Vec3 vp =  (v.T()*ep)*ep;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(vp(0));
          data.push_back(vp(1));
          data.push_back(vp(2));
          data.push_back(0.5);
          openMBVCircularVelocity->append(data);
          data.clear();
          Vec3 vz =  (v.T()*ez)*ez;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(vz(0));
          data.push_back(vz(1));
          data.push_back(vz(2));
          data.push_back(0.5);
          openMBVZVelocity->append(data);
        }

        if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(a(0));
          data.push_back(a(1));
          data.push_back(a(2));
          data.push_back(0.5);
          openMBVAcceleration->append(data);
          data.clear();
          Vec3 ar =  (a.T()*er)*er;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(ar(0));
          data.push_back(ar(1));
          data.push_back(ar(2));
          data.push_back(0.5);
          openMBVRadialAcceleration->append(data);
          data.clear();
          Vec3 ap =  (a.T()*ep)*ep;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(ap(0));
          data.push_back(ap(1));
          data.push_back(ap(2));
          data.push_back(0.5);
          openMBVCircularAcceleration->append(data);
          data.clear();
          Vec3 az =  (a.T()*ez)*ez;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(az(0));
          data.push_back(az(1));
          data.push_back(az(2));
          data.push_back(0.5);
          openMBVZAcceleration->append(data);
        }

        if(openMBVFrame && !openMBVFrame->isHDF5Link()) {
          vector<double> data;
          SqrMat3 AWP;
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

      Observer::plot(t,dt);
    }
  }


}
