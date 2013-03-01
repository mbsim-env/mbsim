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
using namespace fmatvec;

namespace MBSim {

  AbsoluteVelocityObserver::AbsoluteVelocityObserver(const std::string &name) : Observer(name), frame(0) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVArrow=0;
#endif
  }

  void AbsoluteVelocityObserver::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();

      Observer::init(stage);
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVArrow) {
            openMBVArrow->setName(name);
            getOpenMBVGrp()->addObject(openMBVArrow);
          }
        }
#endif
      }
    }
    else
      Observer::init(stage);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void AbsoluteVelocityObserver::enableOpenMBV(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVArrow=new OpenMBV::Arrow;
    openMBVArrow->setScaleLength(scale);
    openMBVArrow->setReferencePoint(refPoint);
    openMBVArrow->setDiameter(diameter);
    openMBVArrow->setHeadDiameter(headDiameter);
    openMBVArrow->setHeadLength(headLength);
    openMBVArrow->setStaticColor(color);
  }
#endif

  void AbsoluteVelocityObserver::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVArrow && !openMBVArrow->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getVelocity()(0));
          data.push_back(frame->getVelocity()(1));
          data.push_back(frame->getVelocity()(2));
          data.push_back(0.5);
          openMBVArrow->append(data);
        }
      }
#endif

      Observer::plot(t,dt);
    }
  }

  AbsoluteKinematicsObserver::AbsoluteKinematicsObserver(const std::string &name) : Observer(name), frame(0) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVPositionArrow=0;
    openMBVVelocityArrow=0;
    openMBVAngularVelocityArrow=0;
    openMBVAccelerationArrow=0;
    openMBVAngularAccelerationArrow=0;
#endif
  }

  void AbsoluteKinematicsObserver::init(InitStage stage) {
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
            openMBVPositionArrow->setName("AbsolutePosition");
            getOpenMBVGrp()->addObject(openMBVPositionArrow);
          }
          if(openMBVVelocityArrow) {
            openMBVVelocityArrow->setName("AbsoluteVelocity");
            getOpenMBVGrp()->addObject(openMBVVelocityArrow);
          }
          if(openMBVAngularVelocityArrow) {
            openMBVAngularVelocityArrow->setName("AbsoluteAngularVelocity");
            getOpenMBVGrp()->addObject(openMBVAngularVelocityArrow);
          }
          if(openMBVAccelerationArrow) {
            openMBVAccelerationArrow->setName("AbsoluteAcceleration");
            getOpenMBVGrp()->addObject(openMBVAccelerationArrow);
          }
          if(openMBVAngularAccelerationArrow) {
            openMBVAngularAccelerationArrow->setName("AbsoluteAngularAcceleration");
            getOpenMBVGrp()->addObject(openMBVAngularAccelerationArrow);
          }
        }
#endif
      }
    }
    else
      Observer::init(stage);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void AbsoluteKinematicsObserver::enableOpenMBVPosition(double diameter, double headDiameter, double headLength, double color) {
    openMBVPositionArrow=new OpenMBV::Arrow;
    openMBVPositionArrow->setReferencePoint(OpenMBV::Arrow::fromPoint);
    openMBVPositionArrow->setDiameter(diameter);
    openMBVPositionArrow->setHeadDiameter(headDiameter);
    openMBVPositionArrow->setHeadLength(headLength);
    openMBVPositionArrow->setStaticColor(color);
  }
  void AbsoluteKinematicsObserver::enableOpenMBVVelocity(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVVelocityArrow=new OpenMBV::Arrow;
    openMBVVelocityArrow->setScaleLength(scale);
    openMBVVelocityArrow->setReferencePoint(refPoint);
    openMBVVelocityArrow->setDiameter(diameter);
    openMBVVelocityArrow->setHeadDiameter(headDiameter);
    openMBVVelocityArrow->setHeadLength(headLength);
    openMBVVelocityArrow->setStaticColor(color);
  }
  void AbsoluteKinematicsObserver::enableOpenMBVAngularVelocity(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVAngularVelocityArrow=new OpenMBV::Arrow;
    openMBVAngularVelocityArrow->setScaleLength(scale);
    openMBVAngularVelocityArrow->setType(OpenMBV::Arrow::toDoubleHead);
    openMBVAngularVelocityArrow->setReferencePoint(refPoint);
    openMBVAngularVelocityArrow->setDiameter(diameter);
    openMBVAngularVelocityArrow->setHeadDiameter(headDiameter);
    openMBVAngularVelocityArrow->setHeadLength(headLength);
    openMBVAngularVelocityArrow->setStaticColor(color);
  }
  void AbsoluteKinematicsObserver::enableOpenMBVAcceleration(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVAccelerationArrow=new OpenMBV::Arrow;
    openMBVAccelerationArrow->setScaleLength(scale);
    openMBVAccelerationArrow->setReferencePoint(refPoint);
    openMBVAccelerationArrow->setDiameter(diameter);
    openMBVAccelerationArrow->setHeadDiameter(headDiameter);
    openMBVAccelerationArrow->setHeadLength(headLength);
    openMBVAccelerationArrow->setStaticColor(color);
  }
  void AbsoluteKinematicsObserver::enableOpenMBVAngularAcceleration(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
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

  void AbsoluteKinematicsObserver::plot(double t, double dt) {
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

  void AbsoluteKinematicsObserver::initializeUsingXML(TiXmlElement *element) {
    cout << "AbsoluteKinematicsObserver" << endl;
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
  }

  RelativeKinematicsObserver::RelativeKinematicsObserver(const std::string &name) : Observer(name) {
    frame[0] = 0;
    frame[1] = 0;
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVr=0;
    openMBVrTrans=0;
    openMBVrRel=0;
    openMBVv=0;
    openMBVvTrans=0;
    openMBVvRot=0;
    openMBVvRel=0;
    openMBVa=0;
    openMBVaTrans=0;
    openMBVaRot=0;
    openMBVaZp=0;
    openMBVaCor=0;
    openMBVaRel=
    openMBVom=0;
    openMBVomTrans=0;
    openMBVomRel=0;
#endif
  }

  void RelativeKinematicsObserver::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();

      Observer::init(stage);
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVr) {
            OpenMBV::Group *openMBVGrp=new OpenMBV::Group();
            openMBVGrp->setName("Position_Group");
            openMBVGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVGrp);
            openMBVr->setName("Position");
            openMBVGrp->addObject(openMBVr);
            openMBVrTrans->setName("Translational_Position");
            openMBVGrp->addObject(openMBVrTrans);
            openMBVrRel->setName("Relative_Position");
            openMBVGrp->addObject(openMBVrRel);
          }
          if(openMBVv) {
            OpenMBV::Group *openMBVGrp=new OpenMBV::Group();
            openMBVGrp->setName("Velocity_Group");
            openMBVGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVGrp);
            openMBVv->setName("Velocity");
            openMBVGrp->addObject(openMBVv);
            openMBVvTrans->setName("Translational_Velocity");
            openMBVGrp->addObject(openMBVvTrans);
            openMBVvRot->setName("Rotational_Velocity");
            openMBVGrp->addObject(openMBVvRot);
            openMBVvRel->setName("Relative_Velocity");
            openMBVGrp->addObject(openMBVvRel);
            openMBVvF->setName("Guiding_Velocity");
            openMBVGrp->addObject(openMBVvF);
          }
          if(openMBVom) {
            OpenMBV::Group *openMBVGrp=new OpenMBV::Group();
            openMBVGrp->setName("Angular_Velocity_Group");
            openMBVGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVGrp);
            openMBVom->setName("Angular_Velocity");
            openMBVGrp->addObject(openMBVom);
            openMBVomTrans->setName("Translational_Angular_Velocity");
            openMBVGrp->addObject(openMBVomTrans);
            openMBVomRel->setName("Relative_Angular_Velocity");
            openMBVGrp->addObject(openMBVomRel);
          }
          if(openMBVa) {
            OpenMBV::Group *openMBVGrp=new OpenMBV::Group();
            openMBVGrp->setName("Acceleration_Group");
            openMBVGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVGrp);
            openMBVa->setName(name+"_Acceleration");
            openMBVGrp->addObject(openMBVa);
            openMBVaTrans->setName("Translational_Acceleration");
            openMBVGrp->addObject(openMBVaTrans);
            openMBVaRot->setName("Rotational_Acceleration");
            openMBVGrp->addObject(openMBVaRot);
            openMBVaZp->setName("Centripetal_Acceleration");
            openMBVGrp->addObject(openMBVaZp);
            openMBVaCor->setName("Coriolis_Acceleration");
            openMBVGrp->addObject(openMBVaCor);
            openMBVaRel->setName("Relative_Acceleration");
            openMBVGrp->addObject(openMBVaRel);
            openMBVaF->setName("Guiding_Acceleration");
            openMBVGrp->addObject(openMBVaF);
          }
          if(openMBVpsi) {
            OpenMBV::Group *openMBVGrp=new OpenMBV::Group();
            openMBVGrp->setName("Angular_Acceleration_Group");
            openMBVGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVGrp);
            openMBVpsi->setName("Angular_Acceleration");
            openMBVGrp->addObject(openMBVpsi);
            openMBVpsiTrans->setName("Translational_Angular_Acceleration");
            openMBVGrp->addObject(openMBVpsiTrans);
            openMBVpsiRot->setName("Rotational_Angular_Acceleration");
            openMBVGrp->addObject(openMBVpsiRot);
            openMBVpsiRel->setName("Relative_Angular_Acceleration");
            openMBVGrp->addObject(openMBVpsiRel);
          }
        }
#endif
      }
    }
    else
      Observer::init(stage);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void RelativeKinematicsObserver::enableOpenMBVPosition(double diameter, double headDiameter, double headLength, double color) {
    openMBVr=new OpenMBV::Arrow;
    openMBVr->setReferencePoint(OpenMBV::Arrow::fromPoint);
    openMBVr->setDiameter(diameter);
    openMBVr->setHeadDiameter(headDiameter);
    openMBVr->setHeadLength(headLength);
    openMBVr->setStaticColor(color);
    openMBVrTrans=new OpenMBV::Arrow;
    openMBVrTrans->setReferencePoint(OpenMBV::Arrow::fromPoint);
    openMBVrTrans->setDiameter(diameter);
    openMBVrTrans->setHeadDiameter(headDiameter);
    openMBVrTrans->setHeadLength(headLength);
    openMBVrTrans->setStaticColor(color);
    openMBVrRel=new OpenMBV::Arrow;
    openMBVrRel->setReferencePoint(OpenMBV::Arrow::fromPoint);
    openMBVrRel->setDiameter(diameter);
    openMBVrRel->setHeadDiameter(headDiameter);
    openMBVrRel->setHeadLength(headLength);
    openMBVrRel->setStaticColor(color);
  }

  void RelativeKinematicsObserver::enableOpenMBVVelocity(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVv=new OpenMBV::Arrow;
    openMBVv->setScaleLength(scale);
    openMBVv->setReferencePoint(refPoint);
    openMBVv->setDiameter(diameter);
    openMBVv->setHeadDiameter(headDiameter);
    openMBVv->setHeadLength(headLength);
    openMBVv->setStaticColor(color);
    openMBVvTrans=new OpenMBV::Arrow;
    openMBVvTrans->setScaleLength(scale);
    openMBVvTrans->setReferencePoint(refPoint);
    openMBVvTrans->setDiameter(diameter);
    openMBVvTrans->setHeadDiameter(headDiameter);
    openMBVvTrans->setHeadLength(headLength);
    openMBVvTrans->setStaticColor(color);
    openMBVvRot=new OpenMBV::Arrow;
    openMBVvRot->setScaleLength(scale);
    openMBVvRot->setReferencePoint(refPoint);
    openMBVvRot->setDiameter(diameter);
    openMBVvRot->setHeadDiameter(headDiameter);
    openMBVvRot->setHeadLength(headLength);
    openMBVvRot->setStaticColor(color);
    openMBVvRel=new OpenMBV::Arrow;
    openMBVvRel->setScaleLength(scale);
    openMBVvRel->setReferencePoint(refPoint);
    openMBVvRel->setDiameter(diameter);
    openMBVvRel->setHeadDiameter(headDiameter);
    openMBVvRel->setHeadLength(headLength);
    openMBVvRel->setStaticColor(color);
    openMBVvF=new OpenMBV::Arrow;
    openMBVvF->setScaleLength(scale);
    openMBVvF->setReferencePoint(refPoint);
    openMBVvF->setDiameter(diameter);
    openMBVvF->setHeadDiameter(headDiameter);
    openMBVvF->setHeadLength(headLength);
    openMBVvF->setStaticColor(color);
  }

  void RelativeKinematicsObserver::enableOpenMBVAcceleration(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVa=new OpenMBV::Arrow;
    openMBVa->setScaleLength(scale);
    openMBVa->setReferencePoint(refPoint);
    openMBVa->setDiameter(diameter);
    openMBVa->setHeadDiameter(headDiameter);
    openMBVa->setHeadLength(headLength);
    openMBVa->setStaticColor(color);
    openMBVaTrans=new OpenMBV::Arrow;
    openMBVaTrans->setScaleLength(scale);
    openMBVaTrans->setReferencePoint(refPoint);
    openMBVaTrans->setDiameter(diameter);
    openMBVaTrans->setHeadDiameter(headDiameter);
    openMBVaTrans->setHeadLength(headLength);
    openMBVaTrans->setStaticColor(color);
    openMBVaRot=new OpenMBV::Arrow;
    openMBVaRot->setScaleLength(scale);
    openMBVaRot->setReferencePoint(refPoint);
    openMBVaRot->setDiameter(diameter);
    openMBVaRot->setHeadDiameter(headDiameter);
    openMBVaRot->setHeadLength(headLength);
    openMBVaRot->setStaticColor(color);
    openMBVaZp=new OpenMBV::Arrow;
    openMBVaZp->setScaleLength(scale);
    openMBVaZp->setReferencePoint(refPoint);
    openMBVaZp->setDiameter(diameter);
    openMBVaZp->setHeadDiameter(headDiameter);
    openMBVaZp->setHeadLength(headLength);
    openMBVaZp->setStaticColor(color);
    openMBVaCor=new OpenMBV::Arrow;
    openMBVaCor->setScaleLength(scale);
    openMBVaCor->setReferencePoint(refPoint);
    openMBVaCor->setDiameter(diameter);
    openMBVaCor->setHeadDiameter(headDiameter);
    openMBVaCor->setHeadLength(headLength);
    openMBVaCor->setStaticColor(color);
    openMBVaRel=new OpenMBV::Arrow;
    openMBVaRel->setScaleLength(scale);
    openMBVaRel->setReferencePoint(refPoint);
    openMBVaRel->setDiameter(diameter);
    openMBVaRel->setHeadDiameter(headDiameter);
    openMBVaRel->setHeadLength(headLength);
    openMBVaRel->setStaticColor(color);
    openMBVaF=new OpenMBV::Arrow;
    openMBVaF->setScaleLength(scale);
    openMBVaF->setReferencePoint(refPoint);
    openMBVaF->setDiameter(diameter);
    openMBVaF->setHeadDiameter(headDiameter);
    openMBVaF->setHeadLength(headLength);
    openMBVaF->setStaticColor(color);
  }

  void RelativeKinematicsObserver::enableOpenMBVAngularVelocity(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVom=new OpenMBV::Arrow;
    openMBVom->setScaleLength(scale);
    openMBVom->setReferencePoint(refPoint);
    openMBVom->setType(OpenMBV::Arrow::toDoubleHead);
    openMBVom->setDiameter(diameter);
    openMBVom->setHeadDiameter(headDiameter);
    openMBVom->setHeadLength(headLength);
    openMBVom->setStaticColor(color);
    openMBVomTrans=new OpenMBV::Arrow;
    openMBVomTrans->setScaleLength(scale);
    openMBVomTrans->setReferencePoint(refPoint);
    openMBVomTrans->setType(OpenMBV::Arrow::toDoubleHead);
    openMBVomTrans->setDiameter(diameter);
    openMBVomTrans->setHeadDiameter(headDiameter);
    openMBVomTrans->setHeadLength(headLength);
    openMBVomTrans->setStaticColor(color);
    openMBVomRel=new OpenMBV::Arrow;
    openMBVomRel->setScaleLength(scale);
    openMBVomRel->setReferencePoint(refPoint);
    openMBVomRel->setType(OpenMBV::Arrow::toDoubleHead);
    openMBVomRel->setDiameter(diameter);
    openMBVomRel->setHeadDiameter(headDiameter);
    openMBVomRel->setHeadLength(headLength);
    openMBVomRel->setStaticColor(color);
  }

  void RelativeKinematicsObserver::enableOpenMBVAngularAcceleration(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVpsi=new OpenMBV::Arrow;
    openMBVpsi->setScaleLength(scale);
    openMBVpsi->setReferencePoint(refPoint);
    openMBVpsi->setType(OpenMBV::Arrow::toDoubleHead);
    openMBVpsi->setDiameter(diameter);
    openMBVpsi->setHeadDiameter(headDiameter);
    openMBVpsi->setHeadLength(headLength);
    openMBVpsi->setStaticColor(color);
    openMBVpsiTrans=new OpenMBV::Arrow;
    openMBVpsiTrans->setScaleLength(scale);
    openMBVpsiTrans->setReferencePoint(refPoint);
    openMBVpsiTrans->setType(OpenMBV::Arrow::toDoubleHead);
    openMBVpsiTrans->setDiameter(diameter);
    openMBVpsiTrans->setHeadDiameter(headDiameter);
    openMBVpsiTrans->setHeadLength(headLength);
    openMBVpsiTrans->setStaticColor(color);
    openMBVpsiRot=new OpenMBV::Arrow;
    openMBVpsiRot->setScaleLength(scale);
    openMBVpsiRot->setReferencePoint(refPoint);
    openMBVpsiRot->setType(OpenMBV::Arrow::toDoubleHead);
    openMBVpsiRot->setDiameter(diameter);
    openMBVpsiRot->setHeadDiameter(headDiameter);
    openMBVpsiRot->setHeadLength(headLength);
    openMBVpsiRot->setStaticColor(color);
    openMBVpsiRel=new OpenMBV::Arrow;
    openMBVpsiRel->setScaleLength(scale);
    openMBVpsiRel->setReferencePoint(refPoint);
    openMBVpsiRel->setType(OpenMBV::Arrow::toDoubleHead);
    openMBVpsiRel->setDiameter(diameter);
    openMBVpsiRel->setHeadDiameter(headDiameter);
    openMBVpsiRel->setHeadLength(headLength);
    openMBVpsiRel->setStaticColor(color);
  }

#endif

  void RelativeKinematicsObserver::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      Vec3 vP = frame[1]->getVelocity();
      Vec3 vOs = frame[0]->getVelocity();
      Vec3 rOP = frame[1]->getPosition();
      Vec3 rOOs = frame[0]->getPosition();
      Vec3 rOsP = rOP - rOOs;
      Vec3 omB = frame[0]->getAngularVelocity();
      Vec3 vOsP = vP - vOs;
      Vec3 vRot = crossProduct(omB,rOsP);
      Vec3 vRel = vOsP - vRot;
      Vec3 vF = vOs + vRot;
      Vec3 aP = frame[1]->getAcceleration();
      Vec3 aOs = frame[0]->getAcceleration();
      Vec3 aOsP = aP - aOs;
      Vec3 psiB = frame[0]->getAngularAcceleration();
      Vec3 aRot = crossProduct(psiB,rOsP);
      Vec3 aZp = crossProduct(omB,crossProduct(omB,rOsP));
      Vec3 aCor = 2.*crossProduct(omB,vRel);
      Vec3 aRel = aOsP - aRot - aZp - aCor;
      Vec3 aF = aOs + aRot + aZp;
      Vec3 omK = frame[1]->getAngularVelocity();
      Vec3 omBK = omK - omB;
      Vec3 psiK = frame[1]->getAngularAcceleration();
      Vec3 psiBK = psiK - psiB;
      Vec3 psiRot = crossProduct(omB, omBK);
      Vec3 psiRel = psiBK - psiRot;

#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVr && !openMBVr->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(0.5);
          openMBVr->append(data);
        }
        if(openMBVrTrans && !openMBVrTrans->isHDF5Link()) {
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
        }
        if(openMBVrRel && !openMBVrRel->isHDF5Link()) {
          vector<double> data;
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
        if(openMBVv && !openMBVv->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(vP(0));
          data.push_back(vP(1));
          data.push_back(vP(2));
          data.push_back(0.5);
          openMBVv->append(data);
        }
        if(openMBVvTrans && !openMBVvTrans->isHDF5Link()) {
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
        }
        if(openMBVvRot && !openMBVvRot->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(vRot(0));
          data.push_back(vRot(1));
          data.push_back(vRot(2));
          data.push_back(0.5);
          openMBVvRot->append(data);
        }
        if(openMBVvRel && !openMBVvRel->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(vRel(0));
          data.push_back(vRel(1));
          data.push_back(vRel(2));
          data.push_back(0.5);
          openMBVvRel->append(data);
        }
        if(openMBVvF && !openMBVvF->isHDF5Link()) {
          vector<double> data;
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
        if(openMBVa && !openMBVa->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aP(0));
          data.push_back(aP(1));
          data.push_back(aP(2));
          data.push_back(0.5);
          openMBVa->append(data);
        }
        if(openMBVaTrans && !openMBVaTrans->isHDF5Link()) {
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
        }
        if(openMBVaRot && !openMBVaRot->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aRot(0));
          data.push_back(aRot(1));
          data.push_back(aRot(2));
          data.push_back(0.5);
          openMBVaRot->append(data);
        }
        if(openMBVaZp && !openMBVaZp->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aZp(0));
          data.push_back(aZp(1));
          data.push_back(aZp(2));
          data.push_back(0.5);
          openMBVaZp->append(data);
        }
        if(openMBVaCor && !openMBVaCor->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aCor(0));
          data.push_back(aCor(1));
          data.push_back(aCor(2));
          data.push_back(0.5);
          openMBVaCor->append(data);
        }
        if(openMBVaRel && !openMBVaRel->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aRel(0));
          data.push_back(aRel(1));
          data.push_back(aRel(2));
          data.push_back(0.5);
          openMBVaRel->append(data);
        }
        if(openMBVaF && !openMBVaF->isHDF5Link()) {
          vector<double> data;
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
        if(openMBVom && !openMBVom->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(omK(0));
          data.push_back(omK(1));
          data.push_back(omK(2));
          data.push_back(0.5);
          openMBVom->append(data);
        }
        if(openMBVomTrans && !openMBVomTrans->isHDF5Link()) {
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
        }
        if(openMBVomRel && !openMBVomRel->isHDF5Link()) {
          vector<double> data;
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
        if(openMBVpsi && !openMBVpsi->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(psiK(0));
          data.push_back(psiK(1));
          data.push_back(psiK(2));
          data.push_back(0.5);
          openMBVpsi->append(data);
        }
        if(openMBVpsiTrans && !openMBVpsiTrans->isHDF5Link()) {
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
        }
        if(openMBVpsiRot && !openMBVpsiRot->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(psiRot(0));
          data.push_back(psiRot(1));
          data.push_back(psiRot(2));
          data.push_back(0.5);
          openMBVpsiRot->append(data);
        }
        if(openMBVpsiRel && !openMBVpsiRel->isHDF5Link()) {
          vector<double> data;
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

      Observer::plot(t,dt);
    }
  }

}
