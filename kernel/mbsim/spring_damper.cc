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
#include "mbsim/spring_damper.h"
#include "mbsim/utils/eps.h"
#include "mbsim/objectfactory.h"
#include "mbsim/fixed_relative_frame.h"
#include "mbsim/rigid_body.h"
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
using namespace boost;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SpringDamper, MBSIM%"SpringDamper")

  SpringDamper::SpringDamper(const string &name) : MechanicalLink(name), func(NULL), l0(0)
  {}

  SpringDamper::~SpringDamper() {
    delete func;
  }

  void SpringDamper::updateForceDirections(double t) {
    if(getGeneralizedRelativePosition(t)(0)>epsroot())
      DF=getGlobalRelativePosition(t)/rrel(0);
    else
      DF.init(0);
    updFD = false;
  }

  void SpringDamper::updateh(double t, int j) {
    h[j][0]-=frame[0]->getJacobianOfTranslation(t,j).T()*getSingleValuedForce(t);
    h[j][1]+=frame[1]->getJacobianOfTranslation(t,j).T()*getSingleValuedForce(t);
  }
  
  void SpringDamper::updatePositions(double t) {
    WrP0P1=frame[1]->getPosition(t) - frame[0]->getPosition(t);
    rrel(0)=nrm2(WrP0P1);
    updPos = false;
  }

  void SpringDamper::updateVelocities(double t) {
    WvP0P1=frame[1]->getVelocity(t) - frame[0]->getVelocity(t);
    vrel=getGlobalForceDirection(t).T()*WvP0P1;
    updVel = false;
  }

  void SpringDamper::updateGeneralizedSingleValuedForces(double t) {
    laSV(0)=-(*func)(getGeneralizedRelativePosition(t)(0)-l0,getGeneralizedRelativeVelocity(t)(0));
    if(rrel(0)<=epsroot() && abs(laSV(0))>epsroot())
      msg(Warn)<<"The SpringDamper force is not 0 and the force direction can not calculated!\nUsing force=0 at t="<<t<<endl;
    updlaSV = false;
  }

  void SpringDamper::connect(Frame *frame0, Frame* frame1) {
    MechanicalLink::connect(frame0);
    MechanicalLink::connect(frame1);
  }

  void SpringDamper::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_ref1!="" && saved_ref2!="")
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      if(not(frame.size()))
        THROW_MBSIMERROR("No connection given!");
      MechanicalLink::init(stage);
    }
    else if(stage==preInit) {
      MechanicalLink::init(stage);
      addDependency(func->getDependency());
    }
    else if(stage==resize) {
      MechanicalLink::init(stage);
      rrel.resize(1);
      vrel.resize(1);
      laSV.resize(1);
      DF.resize(1);
      iF = Index(0, 0);
      iM = Index(0, -1);
    }
    else if(stage==plotting) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(coilspringOpenMBV) {
            coilspringOpenMBV->setName(name);
            parent->getOpenMBVGrp()->addObject(coilspringOpenMBV);
          }
        }
#endif
        MechanicalLink::init(stage);
      }
    }
    else
      MechanicalLink::init(stage);
    func->init(stage);
  }

  void SpringDamper::plot(double t,double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if (coilspringOpenMBV) {
          Vec3 WrOToPoint;
          Vec3 WrOFromPoint;

          WrOFromPoint = frame[0]->getPosition(t);
          WrOToPoint   = frame[1]->getPosition(t);
          vector<double> data;
          data.push_back(t); 
          data.push_back(WrOFromPoint(0));
          data.push_back(WrOFromPoint(1));
          data.push_back(WrOFromPoint(2));
          data.push_back(WrOToPoint(0));
          data.push_back(WrOToPoint(1));
          data.push_back(WrOToPoint(2));
          data.push_back(laSV(0));
          coilspringOpenMBV->append(data);
        }
      }
#endif
      MechanicalLink::plot(t,dt);
    }
  }

  void SpringDamper::initializeUsingXML(DOMElement *element) {
    MechanicalLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
    Function<double(double,double)> *f=ObjectFactory::createAndInit<Function<double(double,double)> >(e->getFirstElementChild());
    setForceFunction(f);
    e = E(element)->getFirstElementChildNamed(MBSIM%"unloadedLength");
    if(e) l0 = Element::getDouble(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"connect");
    saved_ref1=E(e)->getAttribute("ref1");
    saved_ref2=E(e)->getAttribute("ref2");
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVCoilSpring");
    if(e) {
      OpenMBVCoilSpring ombv;
      coilspringOpenMBV=ombv.createOpenMBV(e);
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      setOpenMBVForce(ombv.createOpenMBV(e));
    }
#endif
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(DirectionalSpringDamper, MBSIM%"DirectionalSpringDamper")

  DirectionalSpringDamper::DirectionalSpringDamper(const string &name) : MechanicalLink(name), func(NULL), l0(0), refFrame(NULL), C("F") {
    C.setParent(this);
  }

  DirectionalSpringDamper::~DirectionalSpringDamper() {
    delete func;
  }

  void DirectionalSpringDamper::updateForceDirections(double t) {
    DF=refFrame->getOrientation()*forceDir; // force direction in world system
    updFD = false;
  }

  void DirectionalSpringDamper::updatePositions(double t) {
    WrP0P1=frame[1]->getPosition(t) - frame[0]->getPosition(t);
    C.setPosition(frame[1]->getPosition());
    C.setOrientation(frame[0]->getOrientation());
    rrel=getGlobalForceDirection(t).T()*WrP0P1;
    updPos = false;
  }

  void DirectionalSpringDamper::updateVelocities(double t) {
    WvP0P1=frame[1]->getVelocity(t) - C.getVelocity(t);
    vrel=getGlobalForceDirection(t).T()*WvP0P1;
    updVel = false;
  }

  void DirectionalSpringDamper::updateh(double t, int j) {
    h[j][0]-=frame[0]->getJacobianOfTranslation(t,j).T()*getSingleValuedForce(t);
    h[j][1]+=frame[1]->getJacobianOfTranslation(t,j).T()*getSingleValuedForce(t);
  }

  void DirectionalSpringDamper::updateGeneralizedSingleValuedForces(double t) {
    laSV(0)=-(*func)(getGeneralizedRelativePosition(t)(0)-l0,getGeneralizedRelativeVelocity(t)(0));
    updlaSV = false;
  }

  void DirectionalSpringDamper::connect(Frame *frame0, Frame* frame1) {
    MechanicalLink::connect(frame0);
    MechanicalLink::connect(frame1);
  }

  void DirectionalSpringDamper::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_ref1!="" && saved_ref2!="")
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      if(not(frame.size()))
        THROW_MBSIMERROR("No connection given!");
      MechanicalLink::init(stage);
    }
    else if(stage==preInit) {
      MechanicalLink::init(stage);
      addDependency(func->getDependency());
    }
    else if(stage==resize) {
      MechanicalLink::init(stage);
      rrel.resize(1);
      vrel.resize(1);
      laSV.resize(1);
      DF.resize(1);
      iF = Index(0, 0);
      iM = Index(0, -1);
    }
    else if(stage==plotting) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(coilspringOpenMBV) {
            coilspringOpenMBV->setName(name);
            parent->getOpenMBVGrp()->addObject(coilspringOpenMBV);
          }
        }
#endif
        MechanicalLink::init(stage);
      }
    }
    else if(stage==unknownStage) {
      refFrame=frame[0];
      //C.getJacobianOfTranslation(0).resize(frame[0]->getJacobianOfTranslation(0).cols());
      //C.getJacobianOfRotation(0).resize(frame[0]->getJacobianOfRotation(0).cols());
      //C.getJacobianOfTranslation(1).resize(frame[0]->getJacobianOfTranslation(1).cols());
      //C.getJacobianOfRotation(1).resize(frame[0]->getJacobianOfRotation(1).cols());
      MechanicalLink::init(stage);
    }
    else
      MechanicalLink::init(stage);
    func->init(stage);
  }

  void DirectionalSpringDamper::plot(double t,double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if (coilspringOpenMBV) {
          Vec3 WrOToPoint;
          Vec3 WrOFromPoint;

          WrOFromPoint = C.getPosition();
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
      }
#endif
      MechanicalLink::plot(t,dt);
    }
  }

  void DirectionalSpringDamper::initializeUsingXML(DOMElement *element) {
    MechanicalLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"forceDirection");
    setForceDirection(getVec(e,3));
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
    Function<double(double,double)> *f=ObjectFactory::createAndInit<Function<double(double,double)> >(e->getFirstElementChild());
    setForceFunction(f);
    e = E(element)->getFirstElementChildNamed(MBSIM%"unloadedLength");
    if(e) l0 = Element::getDouble(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"connect");
    saved_ref1=E(e)->getAttribute("ref1");
    saved_ref2=E(e)->getAttribute("ref2");
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVCoilSpring");
    if(e) {
      OpenMBVCoilSpring ombv;
      coilspringOpenMBV=ombv.createOpenMBV(e);
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      setOpenMBVForce(ombv.createOpenMBV(e));
    }
#endif
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedSpringDamper, MBSIM%"GeneralizedSpringDamper")

  GeneralizedSpringDamper::GeneralizedSpringDamper(const string &name) : MechanicalLink(name), func(NULL), l0(0), body(2) {
    h[0].resize(2);
    h[1].resize(2);
    body[0] = 0;
    body[1] = 0;
  }

  GeneralizedSpringDamper::~GeneralizedSpringDamper() {
    delete func;
  }

  void GeneralizedSpringDamper::updatehRef(const Vec &hParent, int j) {
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

  void GeneralizedSpringDamper::updateGeneralizedSingleValuedForces(double t) {
    laSV(0) = -(*func)(getGeneralizedRelativePosition(t)(0)-l0,getGeneralizedRelativeVelocity(t)(0));
    updlaSV = false;
  }

  void GeneralizedSpringDamper::updateForceDirections(double t) {
    DF = body[1]->getFrameOfReference()->getOrientation()*body[1]->getPJT(t);
    DM = body[1]->getFrameOfReference()->getOrientation()*body[1]->getPJR(t);
    updFD = false;
  }

  void GeneralizedSpringDamper::updateh(double t, int j) {
    if(j==0) {
      if(body[0]) h[j][0] -= body[0]->getJRel(t,j).T()*getGeneralizedSingleValuedForce(t);
      h[j][1] += body[1]->getJRel(t,j).T()*getGeneralizedSingleValuedForce(t);
    }
    else {
      h[j][1] += body[1]->getFrameForKinematics()->getJacobianOfTranslation(t,j).T()*getSingleValuedForce(t) + body[1]->getFrameForKinematics()->getJacobianOfRotation(t,j).T()*getSingleValuedMoment(t);
      if(body[0]) {
        h[j][0] -= body[0]->getFrameOfReference()->getJacobianOfTranslation(t,j).T()*getSingleValuedForce(t) + body[0]->getFrameOfReference()->getJacobianOfRotation(t,j).T()*getSingleValuedMoment(t);
      } else {
        h[j][0] -= body[1]->getFrameOfReference()->getJacobianOfTranslation(t,j).T()*getSingleValuedForce(t) + body[1]->getFrameOfReference()->getJacobianOfRotation(t,j).T()*getSingleValuedMoment(t);
      }
    }
  }

  void GeneralizedSpringDamper::updatePositions(double t) {
    rrel=body[0]?(body[1]->getqRel(t)-body[0]->getqRel(t)):body[1]->getqRel(t);
    updPos = false;
  }

  void GeneralizedSpringDamper::updateVelocities(double t) {
    vrel=body[0]?(body[1]->getuRel(t)-body[0]->getuRel(t)):body[1]->getuRel(t);
    updVel = false;
  }

  void GeneralizedSpringDamper::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_body1!="")
        setRigidBodyFirstSide(getByPath<RigidBody>(saved_body1));
      if(saved_body2!="")
        setRigidBodySecondSide(getByPath<RigidBody>(saved_body2));
      if(body[1]==NULL)
        THROW_MBSIMERROR("rigid body on second side must be given!");
      if(body[0]) MechanicalLink::connect(body[0]->getFrameForKinematics());
      MechanicalLink::connect(body[1]->getFrameForKinematics());
      MechanicalLink::init(stage);
    }
    else if(stage==preInit) {
      MechanicalLink::init(stage);
      addDependency(func->getDependency());
    }
    else if(stage==resize) {
      MechanicalLink::init(stage);
      g.resize(1);
      gd.resize(1);
      la.resize(1);
      iF = Index(0, 0);
      iM = Index(0, 0);
      rrel.resize(1);
      vrel.resize(1);
      laSV.resize(1);
    }
    else if(stage==plotting) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
  #ifdef HAVE_OPENMBVCPPINTERFACE
        if(coilspringOpenMBV) {
          coilspringOpenMBV->setName(name);
          parent->getOpenMBVGrp()->addObject(coilspringOpenMBV);
        }
  #endif
        MechanicalLink::init(stage);
      }
    }
    else if(stage==unknownStage) {
      if(body[0] and body[0]->getuRelSize()!=1)
        THROW_MBSIMERROR("rigid body on first side to must have of 1 dof!");
      if(body[1]->getuRelSize()!=1)
        THROW_MBSIMERROR("rigid body on second side must have 1 dof!");
      MechanicalLink::init(stage);
    }
    else
      MechanicalLink::init(stage);
    func->init(stage);
  }

  void GeneralizedSpringDamper::plot(double t,double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (coilspringOpenMBV) {
        Vec WrOToPoint;
        Vec WrOFromPoint;

        WrOFromPoint = body[0]?body[0]->getFrameForKinematics()->getPosition():body[1]->getFrameOfReference()->getPosition();
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
      MechanicalLink::plot(t,dt);
    }
  }

  void GeneralizedSpringDamper::initializeUsingXML(DOMElement *element) {
    MechanicalLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedForceFunction");
    Function<double(double,double)> *f=ObjectFactory::createAndInit<Function<double(double,double)> >(e->getFirstElementChild());
    setGeneralizedForceFunction(f);
    e = E(element)->getFirstElementChildNamed(MBSIM%"unloadedGeneralizedLength");
    if(e) l0 = Element::getDouble(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"rigidBodyFirstSide");
    if(e) saved_body1=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"rigidBodySecondSide");
    saved_body2=E(e)->getAttribute("ref");
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVCoilSpring");
    if(e) {
      OpenMBVCoilSpring ombv;
      coilspringOpenMBV=ombv.createOpenMBV(e);
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      MechanicalLink::setOpenMBVForce(ombv.createOpenMBV(e));
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      MechanicalLink::setOpenMBVMoment(ombv.createOpenMBV(e));
    }
#endif
  }

}
