/* Copyright (C) 2004-2016 MBSim Development Team
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
#include "mbsim/observers/kinematic_coordinates_observer.h"
#include "mbsim/frames/frame.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace xercesc;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, KinematicCoordinatesObserver)

  KinematicCoordinatesObserver::KinematicCoordinatesObserver(const std::string &name) : Observer(name), frame(nullptr), frameOfReference(nullptr) {
    evalOMBVPositionColorRepresentation[0] = &KinematicCoordinatesObserver::evalNone;
    evalOMBVPositionColorRepresentation[1] = &KinematicCoordinatesObserver::evalAbsolutePosition;
    evalOMBVVelocityColorRepresentation[0] = &KinematicCoordinatesObserver::evalNone;
    evalOMBVVelocityColorRepresentation[1] = &KinematicCoordinatesObserver::evalAbsoluteVelocitiy;
    evalOMBVAccelerationColorRepresentation[0] = &KinematicCoordinatesObserver::evalNone;
    evalOMBVAccelerationColorRepresentation[1] = &KinematicCoordinatesObserver::evalAbsoluteAcceleration;
  }

  void KinematicCoordinatesObserver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not saved_frame.empty())
        setFrame(getByPath<Frame>(saved_frame));
      if(not frame)
        throwError("Frame is not given!");
      if(not saved_frameOfReference.empty())
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      if(not frameOfReference)
        setFrameOfReference(static_cast<DynamicSystem*>(parent)->getFrameI());
      Observer::init(stage, config);
    }
    else if(stage==plotting) {
//      if(openMBVPosition) {
//        plotColumns.push_back("Position");
//        plotColumns.push_back("XPosition");
//        plotColumns.push_back("YPosition");
//        plotColumns.push_back("ZPosition");
//      }
//      if(openMBVVelocity) {
//        plotColumns.push_back("Velocity");
//        plotColumns.push_back("XVelocity");
//        plotColumns.push_back("YVelocity");
//        plotColumns.push_back("ZVelocity");
//      }
//      if(openMBVAcceleration) {
//        plotColumns.push_back("Acceleration");
//        plotColumns.push_back("XAcceleration");
//        plotColumns.push_back("YAcceleration");
//        plotColumns.push_back("ZAcceleration");
//      }
      Observer::init(stage, config);
      if(plotFeature[openMBV]) {
        if(ombvPositionArrow) {
          openMBVPosGrp=OpenMBV::ObjectFactory::create<OpenMBV::Group>();
          openMBVPosGrp->setName("Position_Group");
          openMBVPosGrp->setExpand(false);
          getOpenMBVGrp()->addObject(openMBVPosGrp);
          openMBVPosition=ombvPositionArrow->createOpenMBV();
          openMBVPosition->setName("Position");
          openMBVPosGrp->addObject(openMBVPosition);
          openMBVXPosition=ombvPositionArrow->createOpenMBV();
          openMBVXPosition->setName("XPosition");
          openMBVPosGrp->addObject(openMBVXPosition);
          openMBVYPosition=ombvPositionArrow->createOpenMBV();
          openMBVYPosition->setName("YPosition");
          openMBVPosGrp->addObject(openMBVYPosition);
          openMBVZPosition=ombvPositionArrow->createOpenMBV();
          openMBVZPosition->setName("ZPosition");
          openMBVPosGrp->addObject(openMBVZPosition);
        }
        if(ombvVelocityArrow) {
          openMBVVelGrp=OpenMBV::ObjectFactory::create<OpenMBV::Group>();
          openMBVVelGrp->setName("Velocity_Group");
          openMBVVelGrp->setExpand(false);
          getOpenMBVGrp()->addObject(openMBVVelGrp);
          openMBVVelocity=ombvVelocityArrow->createOpenMBV();
          openMBVVelocity->setName("Velocity");
          openMBVVelGrp->addObject(openMBVVelocity);
          openMBVXVelocity=ombvVelocityArrow->createOpenMBV();
          openMBVXVelocity->setName("XVelocity");
          openMBVVelGrp->addObject(openMBVXVelocity);
          openMBVYVelocity=ombvVelocityArrow->createOpenMBV();
          openMBVYVelocity->setName("YVelocity");
          openMBVVelGrp->addObject(openMBVYVelocity);
          openMBVZVelocity=ombvVelocityArrow->createOpenMBV();
          openMBVZVelocity->setName("ZVelocity");
          openMBVVelGrp->addObject(openMBVZVelocity);
        }
        if(ombvAccelerationArrow) {
          openMBVAccGrp=OpenMBV::ObjectFactory::create<OpenMBV::Group>();
          openMBVAccGrp->setName("Acceleration_Group");
          openMBVAccGrp->setExpand(false);
          getOpenMBVGrp()->addObject(openMBVAccGrp);
          openMBVAcceleration=ombvAccelerationArrow->createOpenMBV();
          openMBVAcceleration->setName("Acceleration");
          openMBVAccGrp->addObject(openMBVAcceleration);
          openMBVXAcceleration=ombvAccelerationArrow->createOpenMBV();
          openMBVXAcceleration->setName("XAcceleration");
          openMBVAccGrp->addObject(openMBVXAcceleration);
          openMBVYAcceleration=ombvAccelerationArrow->createOpenMBV();
          openMBVYAcceleration->setName("YAcceleration");
          openMBVAccGrp->addObject(openMBVYAcceleration);
          openMBVZAcceleration=ombvAccelerationArrow->createOpenMBV();
          openMBVZAcceleration->setName("ZAcceleration");
          openMBVAccGrp->addObject(openMBVZAcceleration);
        }
      }
    }
    else
      Observer::init(stage, config);
  }

  void KinematicCoordinatesObserver::plot() {
    if(plotFeature[openMBV]) {
      Vec3 r = frame->evalPosition();
      Vec3 v = frame->evalVelocity();
      Vec3 a = frame->evalAcceleration();
      SqrMat3 A = frameOfReference->evalOrientation();
      Vec3 ex = A.col(0);
      Vec3 ey = A.col(1);
      Vec3 ez = A.col(2);

      if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
        vector<double> data;
        data.push_back(getTime());
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        double cr = (this->*evalOMBVPositionColorRepresentation[ombvPositionArrow->getColorRepresentation()])();
        data.push_back(cr);
        openMBVPosition->append(data);
        //plotVector.push_back(nrm2(r));
        data.clear();
        Vec3 rx =  (r.T()*ex)*ex;
        data.push_back(getTime());
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(rx(0));
        data.push_back(rx(1));
        data.push_back(rx(2));
        data.push_back(cr);
        openMBVXPosition->append(data);
        data.clear();
        Vec3 ry =  (r.T()*ey)*ey;
        data.push_back(getTime());
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(ry(0));
        data.push_back(ry(1));
        data.push_back(ry(2));
        data.push_back(cr);
        openMBVYPosition->append(data);
        data.clear();
        Vec3 rz =  (r.T()*ez)*ez;
        data.push_back(getTime());
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(rz(0));
        data.push_back(rz(1));
        data.push_back(rz(2));
        data.push_back(cr);
        openMBVZPosition->append(data);
        //plotVector.push_back(r(0));
        //plotVector.push_back(r(1));
        //plotVector.push_back(r(2));
      }

      if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
        vector<double> data;
        data.push_back(getTime());
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        data.push_back(v(0));
        data.push_back(v(1));
        data.push_back(v(2));
        double cv = (this->*evalOMBVVelocityColorRepresentation[ombvVelocityArrow->getColorRepresentation()])();
        data.push_back(cv);
        openMBVVelocity->append(data);
        //plotVector.push_back(nrm2(v));
        data.clear();
        Vec3 vx =  (v.T()*ex)*ex;
        data.push_back(getTime());
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        data.push_back(vx(0));
        data.push_back(vx(1));
        data.push_back(vx(2));
        data.push_back(cv);
        openMBVXVelocity->append(data);
        data.clear();
        Vec3 vy =  (v.T()*ey)*ey;
        data.push_back(getTime());
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        data.push_back(vy(0));
        data.push_back(vy(1));
        data.push_back(vy(2));
        data.push_back(cv);
        openMBVYVelocity->append(data);
        data.clear();
        Vec3 vz =  (v.T()*ez)*ez;
        data.push_back(getTime());
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        data.push_back(vz(0));
        data.push_back(vz(1));
        data.push_back(vz(2));
        data.push_back(cv);
        openMBVZVelocity->append(data);
        //plotVector.push_back(v(0));
        //plotVector.push_back(v(1));
        //plotVector.push_back(v(2));
      }

      if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
        vector<double> data;
        data.push_back(getTime());
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        data.push_back(a(0));
        data.push_back(a(1));
        data.push_back(a(2));
        double ca = (this->*evalOMBVAccelerationColorRepresentation[ombvAccelerationArrow->getColorRepresentation()])();
        data.push_back(ca);
        openMBVAcceleration->append(data);
        //plotVector.push_back(nrm2(a));
        data.clear();
        Vec3 ax =  (a.T()*ex)*ex;
        data.push_back(getTime());
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        data.push_back(ax(0));
        data.push_back(ax(1));
        data.push_back(ax(2));
        data.push_back(ca);
        openMBVXAcceleration->append(data);
        data.clear();
        Vec3 ay =  (a.T()*ey)*ey;
        data.push_back(getTime());
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        data.push_back(ay(0));
        data.push_back(ay(1));
        data.push_back(ay(2));
        data.push_back(ca);
        openMBVYAcceleration->append(data);
        data.clear();
        Vec3 az =  (a.T()*ez)*ez;
        data.push_back(getTime());
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        data.push_back(az(0));
        data.push_back(az(1));
        data.push_back(az(2));
        data.push_back(ca);
        openMBVZAcceleration->append(data);
        //plotVector.push_back(a(0));
        //plotVector.push_back(a(1));
        //plotVector.push_back(a(2));
      }
    }
    Observer::plot();
  }

  void KinematicCoordinatesObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"frame");
    saved_frame=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReference");
    if(e) saved_frameOfReference=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVPosition");
    if(e) {
      ombvPositionArrow = shared_ptr<OpenMBVArrow>(new OpenMBVArrow);
      ombvPositionArrow->initializeUsingXML(e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVVelocity");
    if(e) {
      ombvVelocityArrow = shared_ptr<OpenMBVArrow>(new OpenMBVArrow);
      ombvVelocityArrow->initializeUsingXML(e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAcceleration");
    if(e) {
      ombvAccelerationArrow = shared_ptr<OpenMBVArrow>(new OpenMBVArrow);
      ombvAccelerationArrow->initializeUsingXML(e);
    }
  }

  double KinematicCoordinatesObserver::evalAbsolutePosition() {
    return nrm2(frame->evalPosition());
  }

  double KinematicCoordinatesObserver::evalAbsoluteVelocitiy() {
    return nrm2(frame->evalVelocity());
  }

  double KinematicCoordinatesObserver::evalAbsoluteAcceleration() {
    return nrm2(frame->evalAcceleration());
  }

}
