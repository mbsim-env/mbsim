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
#include "mbsim/observer/cartesian_coordinates_observer.h"
#include "mbsim/frame.h"
#include "mbsim/utils/rotarymatrices.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frame.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  CartesianCoordinatesObserver::CartesianCoordinatesObserver(const std::string &name) : Observer(name), frame(0), A(EYE) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVPosition=0;
    openMBVXPosition=0;
    openMBVYPosition=0;
    openMBVZPosition=0;
    openMBVVelocity=0;
    openMBVXVelocity=0;
    openMBVYVelocity=0;
    openMBVZVelocity=0;
    openMBVAcceleration=0;
    openMBVXAcceleration=0;
    openMBVYAcceleration=0;
    openMBVZAcceleration=0;
    openMBVFrame=0;
#endif
  }

  void CartesianCoordinatesObserver::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();

      Observer::init(stage);
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
	ex = A.col(0);
	ey = A.col(1);
	ez = A.col(2);
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVPosition) {
            OpenMBV::Group *openMBVGrp=new OpenMBV::Group();
            openMBVGrp->setName("Position_Group");
            openMBVGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVGrp);
            openMBVPosition->setName("Position");
            openMBVGrp->addObject(openMBVPosition);
            openMBVXPosition->setName("XPosition");
            openMBVGrp->addObject(openMBVXPosition);
            openMBVYPosition->setName("YPosition");
            openMBVGrp->addObject(openMBVYPosition);
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
            openMBVXVelocity->setName("XVelocity");
            openMBVGrp->addObject(openMBVXVelocity);
            openMBVYVelocity->setName("YVelocity");
            openMBVGrp->addObject(openMBVYVelocity);
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
            openMBVXAcceleration->setName("XAcceleration");
            openMBVGrp->addObject(openMBVXAcceleration);
            openMBVYAcceleration->setName("YAcceleration");
            openMBVGrp->addObject(openMBVYAcceleration);
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
  void CartesianCoordinatesObserver::enableOpenMBVPosition(double diameter, double headDiameter, double headLength, double color) {
    openMBVPosition=new OpenMBV::Arrow;
    openMBVPosition->setReferencePoint(OpenMBV::Arrow::fromPoint);
    openMBVPosition->setDiameter(diameter);
    openMBVPosition->setHeadDiameter(headDiameter);
    openMBVPosition->setHeadLength(headLength);
    openMBVPosition->setStaticColor(color);
    openMBVXPosition=new OpenMBV::Arrow;
    openMBVXPosition->setReferencePoint(OpenMBV::Arrow::fromPoint);
    openMBVXPosition->setDiameter(diameter);
    openMBVXPosition->setHeadDiameter(headDiameter);
    openMBVXPosition->setHeadLength(headLength);
    openMBVXPosition->setStaticColor(color);
    openMBVYPosition=new OpenMBV::Arrow;
    openMBVYPosition->setReferencePoint(OpenMBV::Arrow::fromPoint);
    openMBVYPosition->setDiameter(diameter);
    openMBVYPosition->setHeadDiameter(headDiameter);
    openMBVYPosition->setHeadLength(headLength);
    openMBVYPosition->setStaticColor(color);
    openMBVZPosition=new OpenMBV::Arrow;
    openMBVZPosition->setReferencePoint(OpenMBV::Arrow::fromPoint);
    openMBVZPosition->setDiameter(diameter);
    openMBVZPosition->setHeadDiameter(headDiameter);
    openMBVZPosition->setHeadLength(headLength);
    openMBVZPosition->setStaticColor(color);
  }

  void CartesianCoordinatesObserver::enableOpenMBVVelocity(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVVelocity=new OpenMBV::Arrow;
    openMBVVelocity->setScaleLength(scale);
    openMBVVelocity->setReferencePoint(refPoint);
    openMBVVelocity->setDiameter(diameter);
    openMBVVelocity->setHeadDiameter(headDiameter);
    openMBVVelocity->setHeadLength(headLength);
    openMBVVelocity->setStaticColor(color);
    openMBVXVelocity=new OpenMBV::Arrow;
    openMBVXVelocity->setScaleLength(scale);
    openMBVXVelocity->setReferencePoint(refPoint);
    openMBVXVelocity->setDiameter(diameter);
    openMBVXVelocity->setHeadDiameter(headDiameter);
    openMBVXVelocity->setHeadLength(headLength);
    openMBVXVelocity->setStaticColor(color);
    openMBVYVelocity=new OpenMBV::Arrow;
    openMBVYVelocity->setScaleLength(scale);
    openMBVYVelocity->setReferencePoint(refPoint);
    openMBVYVelocity->setDiameter(diameter);
    openMBVYVelocity->setHeadDiameter(headDiameter);
    openMBVYVelocity->setHeadLength(headLength);
    openMBVYVelocity->setStaticColor(color);
    openMBVZVelocity=new OpenMBV::Arrow;
    openMBVZVelocity->setScaleLength(scale);
    openMBVZVelocity->setReferencePoint(refPoint);
    openMBVZVelocity->setDiameter(diameter);
    openMBVZVelocity->setHeadDiameter(headDiameter);
    openMBVZVelocity->setHeadLength(headLength);
    openMBVZVelocity->setStaticColor(color);
  }

  void CartesianCoordinatesObserver::enableOpenMBVAcceleration(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVAcceleration=new OpenMBV::Arrow;
    openMBVAcceleration->setScaleLength(scale);
    openMBVAcceleration->setReferencePoint(refPoint);
    openMBVAcceleration->setDiameter(diameter);
    openMBVAcceleration->setHeadDiameter(headDiameter);
    openMBVAcceleration->setHeadLength(headLength);
    openMBVAcceleration->setStaticColor(color);
    openMBVXAcceleration=new OpenMBV::Arrow;
    openMBVXAcceleration->setScaleLength(scale);
    openMBVXAcceleration->setReferencePoint(refPoint);
    openMBVXAcceleration->setDiameter(diameter);
    openMBVXAcceleration->setHeadDiameter(headDiameter);
    openMBVXAcceleration->setHeadLength(headLength);
    openMBVXAcceleration->setStaticColor(color);
    openMBVYAcceleration=new OpenMBV::Arrow;
    openMBVYAcceleration->setScaleLength(scale);
    openMBVYAcceleration->setReferencePoint(refPoint);
    openMBVYAcceleration->setDiameter(diameter);
    openMBVYAcceleration->setHeadDiameter(headDiameter);
    openMBVYAcceleration->setHeadLength(headLength);
    openMBVYAcceleration->setStaticColor(color);
    openMBVZAcceleration=new OpenMBV::Arrow;
    openMBVZAcceleration->setScaleLength(scale);
    openMBVZAcceleration->setReferencePoint(refPoint);
    openMBVZAcceleration->setDiameter(diameter);
    openMBVZAcceleration->setHeadDiameter(headDiameter);
    openMBVZAcceleration->setHeadLength(headLength);
    openMBVZAcceleration->setStaticColor(color);
  }

  void CartesianCoordinatesObserver::enableOpenMBVFrame(double size, double offset) {
    openMBVFrame=new OpenMBV::Frame;
    openMBVFrame->setSize(size);
    openMBVFrame->setOffset(offset);
  }
#endif

  void CartesianCoordinatesObserver::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        Vec3 r = frame->getPosition();
        Vec3 v = frame->getVelocity();
        Vec3 a = frame->getAcceleration();

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
          Vec3 rx =  (r.T()*ex)*ex;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(rx(0));
          data.push_back(rx(1));
          data.push_back(rx(2));
          data.push_back(0.5);
          openMBVXPosition->append(data);
          data.clear();
          Vec3 ry =  (r.T()*ey)*ey;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(ry(0));
          data.push_back(ry(1));
          data.push_back(ry(2));
          data.push_back(0.5);
          openMBVYPosition->append(data);
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
          Vec3 vx =  (v.T()*ex)*ex;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(vx(0));
          data.push_back(vx(1));
          data.push_back(vx(2));
          data.push_back(0.5);
          openMBVXVelocity->append(data);
          data.clear();
          Vec3 vy =  (v.T()*ey)*ey;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(vy(0));
          data.push_back(vy(1));
          data.push_back(vy(2));
          data.push_back(0.5);
          openMBVYVelocity->append(data);
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
          Vec3 ax =  (a.T()*ex)*ex;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(ax(0));
          data.push_back(ax(1));
          data.push_back(ax(2));
          data.push_back(0.5);
          openMBVXAcceleration->append(data);
          data.clear();
          Vec3 ay =  (a.T()*ey)*ey;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(ay(0));
          data.push_back(ay(1));
          data.push_back(ay(2));
          data.push_back(0.5);
          openMBVYAcceleration->append(data);
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
          AWP.set(0, ex);
          AWP.set(1, ey);
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
