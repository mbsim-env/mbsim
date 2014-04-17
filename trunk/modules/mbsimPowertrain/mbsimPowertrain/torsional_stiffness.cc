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
 * Contact: martin.o.foerg@googlemail.com
 */

#include "config.h"
#include "mbsimPowertrain/torsional_stiffness.h"
#include "mbsimPowertrain/defines.h"
#include "mbsim/utils/eps.h"
#include "mbsim/objectfactory.h"
#include "mbsim/frame.h"
#include "mbsim/rigid_body.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/coilspring.h>
#include <openmbvcppinterface/arrow.h>
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/objectfactory.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimPowertrain {

  const MBXMLUtils::NamespaceURI MBSIMPOWERTRAIN("http://mbsim.berlios.de/MBSimPowertrain");

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, TorsionalStiffness, MBSIMPOWERTRAIN%"TorsionalStiffness")

  TorsionalStiffness::TorsionalStiffness(const string &name) : LinkMechanics(name), func(NULL), body(2)
#ifdef HAVE_OPENMBVCPPINTERFACE
    , coilspringOpenMBV(NULL)
#endif
  {
    WF.resize(2);
    WM.resize(2);
    h[0].resize(2);
    h[1].resize(2);
  }

  void TorsionalStiffness::updatehRef(const Vec &hParent, int j) {
    for(int i=0; i<body.size(); i++) {
      Index I = Index(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      h[j][i]>>hParent(I);
    }
  } 

  void TorsionalStiffness::updateh(double t, int j) {
    la(0) = (*func)(g(0),gd(0));
    if(j==0) {
      h[j][0]+=body[0]->getJRel(j).T()*la;
      h[j][1]-=body[1]->getJRel(j).T()*la;
    }
    else {
        WF[0] = -body[0]->getFrameOfReference()->getOrientation()*body[0]->getPJT()*la;
        WM[0] = -body[0]->getFrameOfReference()->getOrientation()*body[0]->getPJR()*la;
        h[j][0]-=body[0]->getFrameForKinematics()->getJacobianOfTranslation(j).T()*WF[0] + body[0]->getFrameForKinematics()->getJacobianOfRotation(j).T()*WM[0];
        WF[1] = body[1]->getFrameOfReference()->getOrientation()*body[1]->getPJT()*la;
        WM[1] = body[1]->getFrameOfReference()->getOrientation()*body[1]->getPJR()*la;
        h[j][1]-=body[1]->getFrameForKinematics()->getJacobianOfTranslation(j).T()*WF[1] + body[1]->getFrameForKinematics()->getJacobianOfRotation(j).T()*WM[1];
    }
  }

  void TorsionalStiffness::updateg(double) {
    g=body[1]->getqRel()-body[0]->getqRel();
  } 

  void TorsionalStiffness::updategd(double) {
    gd=body[1]->getuRel()-body[0]->getuRel();
  }

  void TorsionalStiffness::init(InitStage stage) {
    assert(body->getRotation()!=NULL);
    if(stage==resolveXMLPath) {
      if(saved_body1!="")
        setRigidBodyFirstSide(getByPath<RigidBody>(saved_body1));
      if(saved_body2!="")
        setRigidBodySecondSide(getByPath<RigidBody>(saved_body2));
      LinkMechanics::connect(body[0]->getFrameForKinematics());
      LinkMechanics::connect(body[1]->getFrameForKinematics());
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

  void TorsionalStiffness::plot(double t,double dt) {
    plotVector.push_back(la(0));
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (coilspringOpenMBV) {
        Vec WrOToPoint;
        Vec WrOFromPoint;

        WrOFromPoint = body[0]->getFrameForKinematics()->getPosition();
        WrOToPoint   = body[1]->getFrameForKinematics()->getPosition();
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

  void TorsionalStiffness::initializeUsingXML(DOMElement *element) {
    LinkMechanics::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMPOWERTRAIN%"generalizedForceFunction");
    Function<double(double,double)> *f=ObjectFactory<FunctionBase>::createAndInit<Function<double(double,double)> >(e->getFirstElementChild());
    setGeneralizedForceFunction(f);
    e=E(element)->getFirstElementChildNamed(MBSIMPOWERTRAIN%"rigidBodyFirstSide");
    saved_body1=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMPOWERTRAIN%"rigidBodySecondSide");
    saved_body2=E(e)->getAttribute("ref");
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=E(element)->getFirstElementChildNamed(MBSIMPOWERTRAIN%"enableOpenMBVCoilSpring");
    if(e) {
      OpenMBVCoilSpring ombv;
      coilspringOpenMBV=ombv.createOpenMBV(e);
    }
    e = E(element)->getFirstElementChildNamed(MBSIMPOWERTRAIN%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      std::vector<bool> which; which.resize(2, true);
      LinkMechanics::setOpenMBVForceArrow(ombv.createOpenMBV(e), which);
    }
    e = E(element)->getFirstElementChildNamed(MBSIMPOWERTRAIN%"enableOpenMBVMoment");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      std::vector<bool> which; which.resize(2, true);
      LinkMechanics::setOpenMBVMomentArrow(ombv.createOpenMBV(e), which);
    }
#endif
  }

}
