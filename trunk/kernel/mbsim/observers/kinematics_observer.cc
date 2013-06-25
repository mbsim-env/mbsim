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

#include <config.h>
#include "mbsim/observers/kinematics_observer.h"
#include "mbsim/frame.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {


  KinematicsObserver::KinematicsObserver(const std::string &name) : Observer(name), frame(0) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVPositionArrow=0;
    openMBVVelocityArrow=0;
    openMBVAngularVelocityArrow=0;
    openMBVAccelerationArrow=0;
    openMBVAngularAccelerationArrow=0;
#endif
  }

  void KinematicsObserver::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frame!="")
        setFrame(getByPath<Frame>(saved_frame));
      Observer::init(stage);
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();

      Observer::init(stage);
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVPositionArrow) {
            openMBVPosGrp=new OpenMBV::Group();
            openMBVPosGrp->setName("Position_Group");
            openMBVPosGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVPosGrp);
            openMBVPositionArrow->setName("AbsolutePosition");
            openMBVPosGrp->addObject(openMBVPositionArrow);
          }
          if(openMBVVelocityArrow) {
            openMBVVelGrp=new OpenMBV::Group();
            openMBVVelGrp->setName("Velocity_Group");
            openMBVVelGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVVelGrp);
            openMBVVelocityArrow->setName("AbsoluteVelocity");
            openMBVVelGrp->addObject(openMBVVelocityArrow);
          }
          if(openMBVAngularVelocityArrow) {
            openMBVAngVelGrp=new OpenMBV::Group();
            openMBVAngVelGrp->setName("Angular_Velocity_Group");
            openMBVAngVelGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVAngVelGrp);
            openMBVAngularVelocityArrow->setName("AbsoluteAngularVelocity");
            openMBVAngVelGrp->addObject(openMBVAngularVelocityArrow);
          }
          if(openMBVAccelerationArrow) {
            openMBVAccGrp=new OpenMBV::Group();
            openMBVAccGrp->setName("Acceleration_Group");
            openMBVAccGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVAccGrp);
            openMBVAccelerationArrow->setName("AbsoluteAcceleration");
            openMBVAccGrp->addObject(openMBVAccelerationArrow);
          }
          if(openMBVAngularAccelerationArrow) {
            openMBVAngAccGrp=new OpenMBV::Group();
            openMBVAngAccGrp->setName("Angular_Acceleration_Group");
            openMBVAngAccGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVAngAccGrp);
            openMBVAngularAccelerationArrow->setName("AbsoluteAngularAcceleration");
            openMBVAngAccGrp->addObject(openMBVAngularAccelerationArrow);
          }
        }
#endif
      }
    }
    else
      Observer::init(stage);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void KinematicsObserver::enableOpenMBVPosition(double diameter, double headDiameter, double headLength, double color) {
    openMBVPositionArrow=new OpenMBV::Arrow;
    openMBVPositionArrow->setReferencePoint(OpenMBV::Arrow::fromPoint);
    openMBVPositionArrow->setDiameter(diameter);
    openMBVPositionArrow->setHeadDiameter(headDiameter);
    openMBVPositionArrow->setHeadLength(headLength);
    openMBVPositionArrow->setStaticColor(color);
  }
  void KinematicsObserver::enableOpenMBVVelocity(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVVelocityArrow=new OpenMBV::Arrow;
    openMBVVelocityArrow->setScaleLength(scale);
    openMBVVelocityArrow->setReferencePoint(refPoint);
    openMBVVelocityArrow->setDiameter(diameter);
    openMBVVelocityArrow->setHeadDiameter(headDiameter);
    openMBVVelocityArrow->setHeadLength(headLength);
    openMBVVelocityArrow->setStaticColor(color);
  }
  void KinematicsObserver::enableOpenMBVAngularVelocity(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVAngularVelocityArrow=new OpenMBV::Arrow;
    openMBVAngularVelocityArrow->setScaleLength(scale);
    openMBVAngularVelocityArrow->setType(OpenMBV::Arrow::toDoubleHead);
    openMBVAngularVelocityArrow->setReferencePoint(refPoint);
    openMBVAngularVelocityArrow->setDiameter(diameter);
    openMBVAngularVelocityArrow->setHeadDiameter(headDiameter);
    openMBVAngularVelocityArrow->setHeadLength(headLength);
    openMBVAngularVelocityArrow->setStaticColor(color);
  }
  void KinematicsObserver::enableOpenMBVAcceleration(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVAccelerationArrow=new OpenMBV::Arrow;
    openMBVAccelerationArrow->setScaleLength(scale);
    openMBVAccelerationArrow->setReferencePoint(refPoint);
    openMBVAccelerationArrow->setDiameter(diameter);
    openMBVAccelerationArrow->setHeadDiameter(headDiameter);
    openMBVAccelerationArrow->setHeadLength(headLength);
    openMBVAccelerationArrow->setStaticColor(color);
  }
  void KinematicsObserver::enableOpenMBVAngularAcceleration(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVAngularAccelerationArrow=new OpenMBV::Arrow;
    openMBVAngularAccelerationArrow->setScaleLength(scale);
    openMBVAngularAccelerationArrow->setType(OpenMBV::Arrow::toDoubleHead);
    openMBVAngularAccelerationArrow->setReferencePoint(refPoint);
    openMBVAngularAccelerationArrow->setDiameter(diameter);
    openMBVAngularAccelerationArrow->setHeadDiameter(headDiameter);
    openMBVAngularAccelerationArrow->setHeadLength(headLength);
    openMBVAngularAccelerationArrow->setStaticColor(color);
  }
#endif

  void KinematicsObserver::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVPositionArrow && !openMBVPositionArrow->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(0.5);
          openMBVPositionArrow->append(data);
        }
        if(openMBVVelocityArrow && !openMBVVelocityArrow->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getVelocity()(0));
          data.push_back(frame->getVelocity()(1));
          data.push_back(frame->getVelocity()(2));
          data.push_back(0.5);
          openMBVVelocityArrow->append(data);
        }
        if(openMBVAngularVelocityArrow && !openMBVAngularVelocityArrow->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getAngularVelocity()(0));
          data.push_back(frame->getAngularVelocity()(1));
          data.push_back(frame->getAngularVelocity()(2));
          data.push_back(0.5);
          openMBVAngularVelocityArrow->append(data);
        }
        if(openMBVAccelerationArrow && !openMBVAccelerationArrow->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getAcceleration()(0));
          data.push_back(frame->getAcceleration()(1));
          data.push_back(frame->getAcceleration()(2));
          data.push_back(0.5);
          openMBVAccelerationArrow->append(data);
        }
        if(openMBVAngularAccelerationArrow && !openMBVAngularAccelerationArrow->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getAngularAcceleration()(0));
          data.push_back(frame->getAngularAcceleration()(1));
          data.push_back(frame->getAngularAcceleration()(2));
          data.push_back(0.5);
          openMBVAngularAccelerationArrow->append(data);
        }
      }
#endif

      Observer::plot(t,dt);
    }
  }

  void KinematicsObserver::initializeUsingXML(TiXmlElement *element) {
    Observer::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"frame");
    if(e) saved_frame=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMNS"enableOpenMBVPosition");
    if(e) {
      enableOpenMBVPosition(getDouble(e->FirstChildElement(MBSIMNS"diameter")),
          getDouble(e->FirstChildElement(MBSIMNS"headDiameter")),
          getDouble(e->FirstChildElement(MBSIMNS"headLength")),
          getDouble(e->FirstChildElement(MBSIMNS"staticColor")));
    }
    e=element->FirstChildElement(MBSIMNS"openMBVPositionArrow");
    if(e) {
      OpenMBV::Arrow *a=new OpenMBV::Arrow;
      setOpenMBVPositionArrow(a);
      a->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"openMBVVelocityArrow");
    if(e) {
      OpenMBV::Arrow *a=new OpenMBV::Arrow;
      setOpenMBVVelocityArrow(a);
      a->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"openMBVAngularVelocityArrow");
    if(e) {
      OpenMBV::Arrow *a=new OpenMBV::Arrow;
      setOpenMBVAngularVelocityArrow(a);
      a->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"openMBVAccelerationArrow");
    if(e) {
      OpenMBV::Arrow *a=new OpenMBV::Arrow;
      setOpenMBVAccelerationArrow(a);
      a->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"openMBVAngularAccelerationArrow");
    if(e) {
      OpenMBV::Arrow *a=new OpenMBV::Arrow;
      setOpenMBVAngularAccelerationArrow(a);
      a->initializeUsingXML(e->FirstChildElement());
    }
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, AbsoluteKinematicsObserver, MBSIMNS"AbsoluteKinematicsObserver")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, RelativeKinematicsObserver, MBSIMNS"RelativeKinematicsObserver")

  RelativeKinematicsObserver::RelativeKinematicsObserver(const std::string &name) : KinematicsObserver(name) {
    refFrame = 0;
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVrTrans=0;
    openMBVrRel=0;
    openMBVvTrans=0;
    openMBVvRot=0;
    openMBVvRel=0;
    openMBVvF=0;
    openMBVaTrans=0;
    openMBVaRot=0;
    openMBVaZp=0;
    openMBVaCor=0;
    openMBVaRel=0;
    openMBVaF=0;
    openMBVomTrans=0;
    openMBVomRel=0;
    openMBVpsiTrans=0;
    openMBVpsiRot=0;
    openMBVpsiRel=0;
#endif
  }

  void RelativeKinematicsObserver::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      KinematicsObserver::init(stage);
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();

      KinematicsObserver::init(stage);
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVPositionArrow) {
            openMBVrTrans = new OpenMBV::Arrow(*openMBVPositionArrow);
            openMBVrRel = new OpenMBV::Arrow(*openMBVPositionArrow);
            openMBVrTrans->setName("Translational_Position");
            openMBVPosGrp->addObject(openMBVrTrans);
            openMBVrRel->setName("Relative_Position");
            openMBVPosGrp->addObject(openMBVrRel);
          }
          if(openMBVVelocityArrow) {
            openMBVvTrans = new OpenMBV::Arrow(*openMBVVelocityArrow);
            openMBVvRot = new OpenMBV::Arrow(*openMBVVelocityArrow);
            openMBVvRel = new OpenMBV::Arrow(*openMBVVelocityArrow);
            openMBVvF = new OpenMBV::Arrow(*openMBVVelocityArrow);
            openMBVvTrans->setName("Translational_Velocity");
            openMBVVelGrp->addObject(openMBVvTrans);
            openMBVvRot->setName("Rotational_Velocity");
            openMBVVelGrp->addObject(openMBVvRot);
            openMBVvRel->setName("Relative_Velocity");
            openMBVVelGrp->addObject(openMBVvRel);
            openMBVvF->setName("Guiding_Velocity");
            openMBVVelGrp->addObject(openMBVvF);
          }
          if(openMBVAngularVelocityArrow) {
            openMBVomTrans = new OpenMBV::Arrow(*openMBVAngularVelocityArrow);
            openMBVomRel = new OpenMBV::Arrow(*openMBVAngularVelocityArrow);
            openMBVomTrans->setName("Translational_Angular_Velocity");
            openMBVAngVelGrp->addObject(openMBVomTrans);
            openMBVomRel->setName("Relative_Angular_Velocity");
            openMBVAngVelGrp->addObject(openMBVomRel);
          }
          if(openMBVAccelerationArrow) {
            openMBVaTrans = new OpenMBV::Arrow(*openMBVAccelerationArrow);
            openMBVaRot = new OpenMBV::Arrow(*openMBVAccelerationArrow);
            openMBVaZp = new OpenMBV::Arrow(*openMBVAccelerationArrow);
            openMBVaCor = new OpenMBV::Arrow(*openMBVAccelerationArrow);
            openMBVaRel = new OpenMBV::Arrow(*openMBVAccelerationArrow);
            openMBVaF = new OpenMBV::Arrow(*openMBVAccelerationArrow);
            openMBVaTrans->setName("Translational_Acceleration");
            openMBVAccGrp->addObject(openMBVaTrans);
            openMBVaRot->setName("Rotational_Acceleration");
            openMBVAccGrp->addObject(openMBVaRot);
            openMBVaZp->setName("Centripetal_Acceleration");
            openMBVAccGrp->addObject(openMBVaZp);
            openMBVaCor->setName("Coriolis_Acceleration");
            openMBVAccGrp->addObject(openMBVaCor);
            openMBVaRel->setName("Relative_Acceleration");
            openMBVAccGrp->addObject(openMBVaRel);
            openMBVaF->setName("Guiding_Acceleration");
            openMBVAccGrp->addObject(openMBVaF);
          }
          if(openMBVAngularAccelerationArrow) {
            openMBVpsiTrans = new OpenMBV::Arrow(*openMBVAngularAccelerationArrow);
            openMBVpsiRot = new OpenMBV::Arrow(*openMBVAngularAccelerationArrow);
            openMBVpsiRel = new OpenMBV::Arrow(*openMBVAngularAccelerationArrow);
            openMBVpsiTrans->setName("Translational_Angular_Acceleration");
            openMBVAngAccGrp->addObject(openMBVpsiTrans);
            openMBVpsiRot->setName("Rotational_Angular_Acceleration");
            openMBVAngAccGrp->addObject(openMBVpsiRot);
            openMBVpsiRel->setName("Relative_Angular_Acceleration");
            openMBVAngAccGrp->addObject(openMBVpsiRel);
          }
        }
#endif
      }
    }
    else
      KinematicsObserver::init(stage);
  }

  void RelativeKinematicsObserver::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      Vec3 vP = frame->getVelocity();
      Vec3 vOs = refFrame->getVelocity();
      Vec3 rOP = frame->getPosition();
      Vec3 rOOs = refFrame->getPosition();
      Vec3 rOsP = rOP - rOOs;
      Vec3 omB = refFrame->getAngularVelocity();
      Vec3 vOsP = vP - vOs;
      Vec3 vRot = crossProduct(omB,rOsP);
      Vec3 vRel = vOsP - vRot;
      Vec3 vF = vOs + vRot;
      Vec3 aP = frame->getAcceleration();
      Vec3 aOs = refFrame->getAcceleration();
      Vec3 aOsP = aP - aOs;
      Vec3 psiB = refFrame->getAngularAcceleration();
      Vec3 aRot = crossProduct(psiB,rOsP);
      Vec3 aZp = crossProduct(omB,crossProduct(omB,rOsP));
      Vec3 aCor = 2.*crossProduct(omB,vRel);
      Vec3 aRel = aOsP - aRot - aZp - aCor;
      Vec3 aF = aOs + aRot + aZp;
      Vec3 omK = frame->getAngularVelocity();
      Vec3 omBK = omK - omB;
      Vec3 psiK = frame->getAngularAcceleration();
      Vec3 psiBK = psiK - psiB;
      Vec3 psiRot = crossProduct(omB, omBK);
      Vec3 psiRel = psiBK - psiRot;

#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVPositionArrow && !openMBVPositionArrow->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(0.5);
          openMBVrTrans->append(data);
          data.clear();
          data.push_back(t);
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(rOsP(0));
          data.push_back(rOsP(1));
          data.push_back(rOsP(2));
          data.push_back(0.5);
          openMBVrRel->append(data);
        }
        if(openMBVVelocityArrow && !openMBVVelocityArrow->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(vOs(0));
          data.push_back(vOs(1));
          data.push_back(vOs(2));
          data.push_back(0.5);
          openMBVvTrans->append(data);
          data.clear();
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(vRot(0));
          data.push_back(vRot(1));
          data.push_back(vRot(2));
          data.push_back(0.5);
          openMBVvRot->append(data);
          data.clear();
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(vRel(0));
          data.push_back(vRel(1));
          data.push_back(vRel(2));
          data.push_back(0.5);
          openMBVvRel->append(data);
          data.clear();
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(vF(0));
          data.push_back(vF(1));
          data.push_back(vF(2));
          data.push_back(0.5);
          openMBVvF->append(data);
        }
        if(openMBVAccelerationArrow && !openMBVAccelerationArrow->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(aOs(0));
          data.push_back(aOs(1));
          data.push_back(aOs(2));
          data.push_back(0.5);
          openMBVaTrans->append(data);
          data.clear();
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aRot(0));
          data.push_back(aRot(1));
          data.push_back(aRot(2));
          data.push_back(0.5);
          openMBVaRot->append(data);
          data.clear();
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aZp(0));
          data.push_back(aZp(1));
          data.push_back(aZp(2));
          data.push_back(0.5);
          openMBVaZp->append(data);
          data.clear();
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aCor(0));
          data.push_back(aCor(1));
          data.push_back(aCor(2));
          data.push_back(0.5);
          openMBVaCor->append(data);
          data.clear();
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aRel(0));
          data.push_back(aRel(1));
          data.push_back(aRel(2));
          data.push_back(0.5);
          openMBVaRel->append(data);
          data.clear();
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aF(0));
          data.push_back(aF(1));
          data.push_back(aF(2));
          data.push_back(0.5);
          openMBVaF->append(data);
        }
        if(openMBVAngularVelocityArrow && !openMBVAngularVelocityArrow->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(omB(0));
          data.push_back(omB(1));
          data.push_back(omB(2));
          data.push_back(0.5);
          openMBVomTrans->append(data);
          data.clear();
          data.push_back(t);
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(omBK(0));
          data.push_back(omBK(1));
          data.push_back(omBK(2));
          data.push_back(0.5);
          openMBVomRel->append(data);
        }
        if(openMBVAngularAccelerationArrow && !openMBVAngularAccelerationArrow->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(psiB(0));
          data.push_back(psiB(1));
          data.push_back(psiB(2));
          data.push_back(0.5);
          openMBVpsiTrans->append(data);
          data.clear();
          data.push_back(t);
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(psiRot(0));
          data.push_back(psiRot(1));
          data.push_back(psiRot(2));
          data.push_back(0.5);
          openMBVpsiRot->append(data);
          data.clear();
          data.push_back(t);
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(psiRel(0));
          data.push_back(psiRel(1));
          data.push_back(psiRel(2));
          data.push_back(0.5);
          openMBVpsiRel->append(data);
        }
      }
#endif

      KinematicsObserver::plot(t,dt);
    }
  }

  void RelativeKinematicsObserver::initializeUsingXML(TiXmlElement *element) {
    KinematicsObserver::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"frameOfReference");
    if(e) saved_frameOfReference=e->Attribute("ref");
  }

}
