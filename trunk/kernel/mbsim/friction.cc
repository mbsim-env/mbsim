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

#include <config.h>
#include "mbsim/friction.h"
#include "mbsim/utils/eps.h"
#include "mbsim/objectfactory.h"
#include "mbsim/frame.h"
#include "mbsim/rigid_body.h"
#include "mbsim/constitutive_laws.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/coilspring.h>
#include <openmbvcppinterface/arrow.h>
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/objectfactory.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedFriction, MBSIM%"GeneralizedFriction")

  GeneralizedFriction::GeneralizedFriction(const string &name) : LinkMechanics(name), func(NULL), laN(0), body(2)
  {
    WF.resize(2);
    WM.resize(2);
    h[0].resize(2);
    h[1].resize(2);
    body[0] = 0;
    body[1] = 0;
  }

  GeneralizedFriction::~GeneralizedFriction() {
    delete func;
  }

  void GeneralizedFriction::updatehRef(const Vec &hParent, int j) {
    Index I = Index(body[1]->gethInd(j),body[1]->gethInd(j)+body[1]->gethSize(j)-1);
    h[j][1]>>hParent(I);
    if(body[0]) {
      Index I = Index(body[0]->gethInd(j),body[0]->gethInd(j)+body[0]->gethSize(j)-1);
      h[j][0]>>hParent(I);
    }
    else {
      Index I = Index(body[1]->getFrameOfReference()->gethInd(j),body[1]->getFrameOfReference()->gethInd(j)+body[1]->getFrameOfReference()->getJacobianOfTranslation(j).cols()-1);
      h[j][0]>>hParent(I);
    }
  } 

  void GeneralizedFriction::updateh(double t, int j) {
    la = -(*func)(gd, (*laN)(t));
    if(j==0) {
      if(body[0]) h[j][0]+=body[0]->getJRel(j).T()*la;
      h[j][1]-=body[1]->getJRel(j).T()*la;
    }
    else {
      WF[1] = body[1]->getFrameOfReference()->getOrientation()*body[1]->getPJT()*la;
      WM[1] = body[1]->getFrameOfReference()->getOrientation()*body[1]->getPJR()*la;
      h[j][1]-=body[1]->getFrameForKinematics()->getJacobianOfTranslation(j).T()*WF[1] + body[1]->getFrameForKinematics()->getJacobianOfRotation(j).T()*WM[1];
      if(body[0]) {
        WF[0] = -body[0]->getFrameOfReference()->getOrientation()*body[0]->getPJT()*la;
        WM[0] = -body[0]->getFrameOfReference()->getOrientation()*body[0]->getPJR()*la;
        h[j][0]-=body[0]->getFrameForKinematics()->getJacobianOfTranslation(j).T()*WF[0] + body[0]->getFrameForKinematics()->getJacobianOfRotation(j).T()*WM[0];
      } else {
        WF[0] = -WF[1];
        WM[0] = -WM[1];
        h[j][0]-=body[1]->getFrameOfReference()->getJacobianOfTranslation(j).T()*WF[0]+body[1]->getFrameOfReference()->getJacobianOfRotation(j).T()*WM[0];
      }
    }
  }

  void GeneralizedFriction::updategd(double) {
    gd=body[0]?(body[1]->getuRel()-body[0]->getuRel()):body[1]->getuRel();
  }

  void GeneralizedFriction::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_body1!="")
        setRigidBodyFirstSide(getByPath<RigidBody>(saved_body1));
      if(saved_body2!="")
        setRigidBodySecondSide(getByPath<RigidBody>(saved_body2));
      if(body[1]==NULL)
        THROW_MBSIMERROR("rigid body on second side must be given!");
      if(body[0]) LinkMechanics::connect(body[0]->getFrameForKinematics());
      LinkMechanics::connect(body[1]->getFrameForKinematics());
      LinkMechanics::init(stage);
    }
    else if(stage==resize) {
      LinkMechanics::init(stage);
      g.resize(1);
      gd.resize(1);
      la.resize(1);
    }
    else if(stage==plotting) {
      updatePlotFeatures();
      plotColumns.push_back("la(0)");
      if(getPlotFeature(plotRecursive)==enabled) {
        LinkMechanics::init(stage);
      }
    }
    else if(stage==unknownStage) {
      if(body[0] and body[0]->getuRelSize()!=1)
        THROW_MBSIMERROR("rigid body on first side to must have of 1 dof!");
      if(body[1]->getuRelSize()!=1)
        THROW_MBSIMERROR("rigid body on second side must have 1 dof!");
      LinkMechanics::init(stage);
    }
    else
      LinkMechanics::init(stage);
    func->init(stage);
  }

  void GeneralizedFriction::setGeneralizedFrictionForceLaw(FrictionForceLaw *func_) { 
    func = func_; 
    func->setParent(this);
  }

  void GeneralizedFriction::plot(double t,double dt) {
    plotVector.push_back(la(0));
    if(getPlotFeature(plotRecursive)==enabled) {
      LinkMechanics::plot(t,dt);
    }
  }

  void GeneralizedFriction::initializeUsingXML(DOMElement *element) {
    LinkMechanics::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedFrictionForceLaw");
    setGeneralizedFrictionForceLaw(ObjectFactory::createAndInit<FrictionForceLaw>(e->getFirstElementChild()));
   //Function<double(double,double)> *f=ObjectFactory::createAndInit<Function<double(double,double)> >(e->getFirstElementChild());
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedNormalForceFunction");
    setGeneralizedNormalForceFunction(ObjectFactory::createAndInit<Function<double(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"rigidBodyFirstSide");
    if(e) saved_body1=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"rigidBodySecondSide");
    saved_body2=E(e)->getAttribute("ref");
#ifdef HAVE_OPENMBVCPPINTERFACE
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      std::vector<bool> which; which.resize(2, true);
      LinkMechanics::setOpenMBVForceArrow(ombv.createOpenMBV(e), which);
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      std::vector<bool> which; which.resize(2, true);
      LinkMechanics::setOpenMBVMomentArrow(ombv.createOpenMBV(e), which);
    }
#endif
  }

}
