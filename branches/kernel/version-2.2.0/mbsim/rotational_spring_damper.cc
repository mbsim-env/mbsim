/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: rzander@users.berlios.de
 */

#include "config.h"
#include "mbsim/rotational_spring_damper.h"
#include "mbsim/rigid_body.h"
#include "mbsim/utils/eps.h"
#include "mbsim/objectfactory.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/coilspring.h>
#include <openmbvcppinterface/arrow.h>
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/objectfactory.h"
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  RelativeRotationalSpringDamper::RelativeRotationalSpringDamper(const string &name) : LinkMechanics(name), func(NULL), refFrame(NULL), body(NULL), torqueDir(3), WtorqueDir(3)
#ifdef HAVE_OPENMBVCPPINTERFACE
    , coilspringOpenMBV(NULL)
#endif
  {}

  void RelativeRotationalSpringDamper::setRelativeBody(RigidBody* body_) {
    body     = body_;
    refFrame = body->getFrameOfReference();
  }

  void RelativeRotationalSpringDamper::updateh(double t) {
    la(0) = (*func)(g(0),gd(0));
    WM[0] =  WtorqueDir*la; // projected force in direction of WtorqueDir
    WM[1] = -WM[0];
    for(unsigned int i=0; i<2; i++) {
      h[i]+=trans(frame[i]->getJacobianOfRotation())*WM[i];
      hLink[i]+=trans(frame[i]->getJacobianOfRotation())*WM[i];
    }
  }

  void RelativeRotationalSpringDamper::updateg(double) {
//    SqrMat       Arel = inv(frame[0]->getOrientation()) * frame[1]->getOrientation();
//    Vec    Womega_rel = frame[1]->getAngularVelocity() - frame[0]->getAngularVelocity();

      WtorqueDir=refFrame->getOrientation()*torqueDir; // force direction in world system
      g=body->getq().copy();
  } 

  void RelativeRotationalSpringDamper::updategd(double) {
//    Vec Womega_rel = frame[1]->getAngularVelocity() - frame[0]->getAngularVelocity();
//    gd(0)=trans(Womega_rel)*WtorqueDir;
      gd=body->getu().copy();
  }

  void RelativeRotationalSpringDamper::connect(Frame *frame0, Frame* frame1) {
    LinkMechanics::connect(frame0);
    LinkMechanics::connect(frame1);
  }

  void RelativeRotationalSpringDamper::init(InitStage stage) {
    assert(body->getRotation()!=NULL);
//    if(stage==resolveXMLPath) {
//      if(saved_frameOfReference!="") {
//        Frame *ref=getFrameByPath(saved_frameOfReference);
//        setProjectionDirection(ref, saved_direction);
//      }
//      if(saved_ref1!="" && saved_ref2!="") {
//        Frame *ref1=getFrameByPath(saved_ref1);
//        Frame *ref2=getFrameByPath(saved_ref2);
//        connect(ref1,ref2);
//      }
//      LinkMechanics::init(stage);
//    }
//    else
    if(stage==resize) {
      LinkMechanics::init(stage);
      g.resize(body->getRotation()->getqSize());
      gd.resize(body->getRotation()->getqSize());
      la.resize(body->getRotation()->getqSize());
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures(parent);
      plotColumns.push_back("la(0)");
      if(getPlotFeature(plotRecursive)==enabled) {
  #ifdef HAVE_OPENMBVCPPINTERFACE
        if(coilspringOpenMBV) {
          coilspringOpenMBV->setName(name);
          parent->getOpenMBVGrp()->addObject(coilspringOpenMBV);
        }
  #endif
        LinkMechanics::init(stage);
      }
    }
    else {
      //Rotation *rot = body->getRotation();
      if(dynamic_cast<RotationAboutFixedAxis*>(body->getRotation())) {
        RotationAboutFixedAxis *rot = dynamic_cast<RotationAboutFixedAxis*>(body->getRotation());
        assert( rot != NULL );
        torqueDir = rot->getAxisOfRotation();
      } else if(dynamic_cast<RotationAboutXAxis*>(body->getRotation())) {
        torqueDir = Vec("[1;0;0]");
      } else if(dynamic_cast<RotationAboutYAxis*>(body->getRotation())) {
        torqueDir = Vec("[0;1;0]");
      } else if(dynamic_cast<RotationAboutZAxis*>(body->getRotation())) {
        torqueDir = Vec("[0;0;1]");
      }
      LinkMechanics::init(stage);
    }
  }

  void RelativeRotationalSpringDamper::plot(double t,double dt) {
    plotVector.push_back(la(0));
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (coilspringOpenMBV) {
        Vec WrOToPoint;
        Vec WrOFromPoint;

        WrOFromPoint = frame[0]->getPosition();
        WrOToPoint   = frame[1]->getPosition();
        vector<double> data;
        data.push_back(t); 
        data.push_back(WrOFromPoint(0));
        data.push_back(WrOFromPoint(1));
        data.push_back(WrOFromPoint(2));
        data.push_back(WrOToPoint(0));
        data.push_back(WrOToPoint(1));
        data.push_back(WrOToPoint(2));
        data.push_back(la(0));
        coilspringOpenMBV->append(data);
      }
#endif
      LinkMechanics::plot(t,dt);
    }
  }

  void RelativeRotationalSpringDamper::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    LinkMechanics::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMNS"forceFunction");
    Function2<double,double,double> *f=ObjectFactory::getInstance()->createFunction2_SSS(e->FirstChildElement());
    setForceFunction(f);
    f->initializeUsingXML(e->FirstChildElement());
    e=element->FirstChildElement(MBSIMNS"projectionDirection");
    if(e) {
      TiXmlElement *ee=e->FirstChildElement(MBSIMNS"frameOfReference");
      saved_frameOfReference=ee->Attribute("ref");
      ee=e->FirstChildElement(MBSIMNS"direction");
      saved_direction=getVec(ee,3);
    }
    e=element->FirstChildElement(MBSIMNS"connect");
    saved_ref1=e->Attribute("ref1");
    saved_ref2=e->Attribute("ref2");
    e=e->NextSiblingElement();
#ifdef HAVE_OPENMBVCPPINTERFACE
    OpenMBV::CoilSpring *coilSpring=dynamic_cast<OpenMBV::CoilSpring*>(OpenMBV::ObjectFactory::createObject(e));
    if(coilSpring) {
      setOpenMBVSpring(coilSpring);
      coilSpring->initializeUsingXML(e);
      e=e->NextSiblingElement();
    }
    OpenMBV::Arrow *arrow=dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(e));
    if(arrow) {
      arrow->initializeUsingXML(e); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
      setOpenMBVForceArrow(arrow);
      e=e->NextSiblingElement();
    }
#endif
  }

}
