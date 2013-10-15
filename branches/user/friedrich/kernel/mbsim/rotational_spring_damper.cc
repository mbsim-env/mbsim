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

  RelativeRotationalSpringDamper::RelativeRotationalSpringDamper(const string &name) : LinkMechanics(name), func(NULL), body(NULL)
#ifdef HAVE_OPENMBVCPPINTERFACE
    , coilspringOpenMBV(NULL)
#endif
  {
    WM.resize(2);
    h[0].resize(2);
    h[1].resize(2);
  }

  void RelativeRotationalSpringDamper::updatehRef(const Vec &hParent, int j) {
    Index I = Index(body->getFrameOfReference()->gethInd(j),body->getFrameOfReference()->gethInd(j)+body->getFrameOfReference()->getJacobianOfTranslation(j).cols()-1);
    h[j][0]>>hParent(I);
    I = Index(body->gethInd(j),body->gethInd(j)+body->gethSize(j)-1);
    h[j][1]>>hParent(I);
  } 

  void RelativeRotationalSpringDamper::updateh(double t, int j) {
    la(0) = (*func)(g(0),gd(0));
    if(j==0)
      h[j][1]-=body->getJRel(j).T()*la;
    else {
      WM[1] = body->getFrameOfReference()->getOrientation()*body->getPJR().col(0)*la; // projected force in direction of WtorqueDir
      WM[0] = -WM[1];
      h[j][0]-=body->getFrameOfReference()->getJacobianOfRotation(j).T()*WM[0];
      h[j][1]-=body->getFrameForKinematics()->getJacobianOfRotation(j).T()*WM[1];
    }
  }

  void RelativeRotationalSpringDamper::updateg(double) {
//    SqrMat       Arel = inv(frame[0]->getOrientation()) * frame[1]->getOrientation();
//    Vec    Womega_rel = frame[1]->getAngularVelocity() - frame[0]->getAngularVelocity();

      g=body->getq();
  } 

  void RelativeRotationalSpringDamper::updategd(double) {
//    Vec Womega_rel = frame[1]->getAngularVelocity() - frame[0]->getAngularVelocity();
//    gd(0)=trans(Womega_rel)*WtorqueDir;
      gd=body->getu();
  }

  void RelativeRotationalSpringDamper::init(InitStage stage) {
    assert(body->getRotation()!=NULL);
    if(stage==resolveXMLPath) {
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
    else
      LinkMechanics::init(stage);
  }

  void RelativeRotationalSpringDamper::plot(double t,double dt) {
    plotVector.push_back(la(0));
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (coilspringOpenMBV) {
        Vec WrOToPoint;
        Vec WrOFromPoint;

        WrOFromPoint = body->getFrameOfReference()->getPosition();
        WrOToPoint   = body->getFrameForKinematics()->getPosition();
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
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"momentFunction");
    Function<double(double,double)> *f=ObjectFactory<FunctionBase>::createAndInit<Function<double(double,double)> >(e->FirstChildElement());
    setMomentFunction(f);
    e=element->FirstChildElement(MBSIMNS"rigidBody");
    saved_body=e->Attribute("ref");
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
