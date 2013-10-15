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
#include "mbsim/frame.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/coilspring.h>
#include <openmbvcppinterface/arrow.h>
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/objectfactory.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, RelativeRotationalSpringDamper, MBSIMNS"RelativeRotationalSpringDamper")

  RelativeRotationalSpringDamper::RelativeRotationalSpringDamper(const string &name) : LinkMechanics(name), func(NULL), refFrame(NULL), body(NULL)
#ifdef HAVE_OPENMBVCPPINTERFACE
    , coilspringOpenMBV(NULL)
#endif
  {}

  void RelativeRotationalSpringDamper::updateh(double t, int j) {
    la(0) = (*func)(g(0),gd(0));
    WM[0] =  WtorqueDir*la; // projected force in direction of WtorqueDir
    WM[1] = -WM[0];
    for(unsigned int i=0; i<2; i++) {
      h[j][i]+=frame[i]->getJacobianOfRotation(j).T()*WM[i];
    }
  }

  void RelativeRotationalSpringDamper::updateg(double) {
//    SqrMat       Arel = inv(frame[0]->getOrientation()) * frame[1]->getOrientation();
//    Vec    Womega_rel = frame[1]->getAngularVelocity() - frame[0]->getAngularVelocity();

      WtorqueDir=refFrame->getOrientation()*body->getPJR().col(0); // force direction in world system
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
    if(stage==resolveXMLPath) {
//      if(saved_frameOfReference!="")
//        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      if(saved_ref1!="" && saved_ref2!="")
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      if(saved_body!="")
        setRigidBody(getByPath<RigidBody>(saved_body));
      LinkMechanics::init(stage);
    }
    else if(stage==resize) {
      LinkMechanics::init(stage);
      g.resize(1);
      gd.resize(1);
      la.resize(1);
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();
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
      if(body) {
        refFrame = body->getFrameOfReference();
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
    LinkMechanics::initializeUsingXML(element);
//    TiXmlElement *e=e->FirstChildElement(MBSIMNS"frameOfReference");
//    if(e) saved_frameOfReference=e->Attribute("ref");
//    e=e->FirstChildElement(MBSIMNS"momentDirection");
//    if(e) setMomentDirection(getVec(e,3));
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"momentFunction");
    Function<double(double,double)> *f=ObjectFactory<FunctionBase>::createAndInit<Function<double(double,double)> >(e->FirstChildElement());
    setMomentFunction(f);
    e=element->FirstChildElement(MBSIMNS"rigidBody");
    saved_body=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMNS"connect");
    saved_ref1=e->Attribute("ref1");
    saved_ref2=e->Attribute("ref2");
    e=e->NextSiblingElement();
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=element->FirstChildElement(MBSIMNS"openMBVCoilSpring");
    if(e) {
      OpenMBV::CoilSpring *coilSpring=OpenMBV::ObjectFactory::create<OpenMBV::CoilSpring>(e->FirstChildElement());
      coilSpring->initializeUsingXML(e->FirstChildElement());
      setOpenMBVCoilSpring(coilSpring);
    }
    e=element->FirstChildElement(MBSIMNS"openMBVMomentArrow");
    if(e) {
      OpenMBV::Arrow *arrow = OpenMBV::ObjectFactory::create<OpenMBV::Arrow>(e->FirstChildElement());
      arrow->initializeUsingXML(e->FirstChildElement()); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
      setOpenMBVMomentArrow(arrow);
    }
#endif
  }

}
