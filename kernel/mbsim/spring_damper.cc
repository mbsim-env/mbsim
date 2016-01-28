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

  SpringDamper::SpringDamper(const string &name) : FrameLink(name), func(NULL), l0(0)
  {}

  SpringDamper::~SpringDamper() {
    delete func;
  }

  void SpringDamper::updateGeneralizedForceForces(double t) {
    lambdaF(0)=-(*func)(getGeneralizedRelativePosition(t)(0)-l0,getGeneralizedRelativeVelocity(t)(0));
    if(rrel(0)<=epsroot() && abs(laSV(0))>epsroot())
      msg(Warn)<<"The SpringDamper force is not 0 and the force direction can not calculated!\nUsing force=0 at t="<<t<<endl;
    updlaF = false;
  }

  void SpringDamper::init(InitStage stage) {
    if(stage==plotting) {
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
        FrameLink::init(stage);
      }
    }
    else
      FrameLink::init(stage);
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
          data.push_back(fabs(getGeneralizedForce(t)(0)));
          coilspringOpenMBV->append(data);
        }
      }
#endif
      FrameLink::plot(t,dt);
    }
  }

  void SpringDamper::initializeUsingXML(DOMElement *element) {
    FrameLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
    Function<double(double,double)> *f=ObjectFactory::createAndInit<Function<double(double,double)> >(e->getFirstElementChild());
    setForceFunction(f);
    e = E(element)->getFirstElementChildNamed(MBSIM%"unloadedLength");
    if(e) l0 = Element::getDouble(e);
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVCoilSpring");
    if(e) {
      OpenMBVCoilSpring ombv;
      coilspringOpenMBV=ombv.createOpenMBV(e);
    }
#endif
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(DirectionalSpringDamper, MBSIM%"DirectionalSpringDamper")

  DirectionalSpringDamper::DirectionalSpringDamper(const string &name) : FloatingFrameLink(name), func(NULL), l0(0) {
  }

  DirectionalSpringDamper::~DirectionalSpringDamper() {
    delete func;
  }

//  void DirectionalSpringDamper::updateForceDirections(double t) {
//    DF=refFrame->getOrientation()*forceDir; // force direction in world system
//    updFD = false;
//  }

  void DirectionalSpringDamper::updatePositions(double t) {
    WrP0P1 = frame[1]->getPosition(t) - frame[0]->getPosition(t);
    C.setPosition(frame[1]->getPosition() - getGlobalForceDirection(t)*(getGlobalForceDirection(t).T()*WrP0P1));;
    C.setOrientation(frame[0]->getOrientation());
    updPos = false;
  }

//  void DirectionalSpringDamper::updateVelocities(double t) {
//    WvP0P1=frame[1]->getVelocity(t) - C.getVelocity(t);
//    updVel = false;
//  }

  void DirectionalSpringDamper::updateGeneralizedPositions(double t) {
    rrel=getGlobalForceDirection(t).T()*getGlobalRelativePosition(t);
    updrrel = false;
  }

  void DirectionalSpringDamper::updateGeneralizedVelocities(double t) {
    vrel=getGlobalForceDirection(t).T()*getGlobalRelativeVelocity(t);
    updvrel = false;
  }

//  void DirectionalSpringDamper::updateh(double t, int j) {
//    h[j][0]-=C.getJacobianOfTranslation(t,j).T()*getSingleValuedForce(t);
//    h[j][1]+=frame[1]->getJacobianOfTranslation(t,j).T()*getSingleValuedForce(t);
//  }

  void DirectionalSpringDamper::updateGeneralizedForceForces(double t) {
    lambdaF(0)=-(*func)(getGeneralizedRelativePosition(t)(0)-l0,getGeneralizedRelativeVelocity(t)(0));
    updlaF = false;
  }

  void DirectionalSpringDamper::init(InitStage stage) {
    if(stage==plotting) {
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
        FloatingFrameLink::init(stage);
      }
    }
    else
      FloatingFrameLink::init(stage);
    func->init(stage);
  }

  void DirectionalSpringDamper::plot(double t,double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if (coilspringOpenMBV) {
          Vec3 WrOToPoint;
          Vec3 WrOFromPoint;

          WrOFromPoint = C.getPosition(t);
          WrOToPoint   = frame[1]->getPosition(t);
          vector<double> data;
          data.push_back(t); 
          data.push_back(WrOFromPoint(0));
          data.push_back(WrOFromPoint(1));
          data.push_back(WrOFromPoint(2));
          data.push_back(WrOToPoint(0));
          data.push_back(WrOToPoint(1));
          data.push_back(WrOToPoint(2));
          data.push_back(fabs(getGeneralizedForce(t)(0)));
          coilspringOpenMBV->append(data);
        }
      }
#endif
      FloatingFrameLink::plot(t,dt);
    }
  }

  void DirectionalSpringDamper::initializeUsingXML(DOMElement *element) {
    FloatingFrameLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"forceDirection");
    setForceDirection(getVec(e,3));
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
    Function<double(double,double)> *f=ObjectFactory::createAndInit<Function<double(double,double)> >(e->getFirstElementChild());
    setForceFunction(f);
    e=E(element)->getFirstElementChildNamed(MBSIM%"unloadedLength");
    if(e) l0 = Element::getDouble(e);
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVCoilSpring");
    if(e) {
      OpenMBVCoilSpring ombv;
      coilspringOpenMBV=ombv.createOpenMBV(e);
    }
#endif
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedSpringDamper, MBSIM%"GeneralizedSpringDamper")

  GeneralizedSpringDamper::GeneralizedSpringDamper(const string &name) : RigidBodyLink(name), func(NULL), l0(0) {
  }

  GeneralizedSpringDamper::~GeneralizedSpringDamper() {
    delete func;
  }

  void GeneralizedSpringDamper::updateGeneralizedSingleValuedForces(double t) {
    laSV(0) = -(*func)(getGeneralizedRelativePosition(t)(0)-l0,getGeneralizedRelativeVelocity(t)(0));
    updlaSV = false;
  }

  void GeneralizedSpringDamper::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_body1!="")
        setRigidBodyFirstSide(getByPath<RigidBody>(saved_body1));
      if(saved_body2!="")
        setRigidBodySecondSide(getByPath<RigidBody>(saved_body2));
      if(body[1]==NULL)
        THROW_MBSIMERROR("rigid body on second side must be given!");
      if(body[0]) connect(body[0]);
      connect(body[1]);
      RigidBodyLink::init(stage);
    }
    else if(stage==resize) {
      RigidBodyLink::init(stage);
      ratio.resize(RigidBodyLink::body.size());
      ratio[0] = -1;
      ratio[ratio.size()-1] = 1;
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
        RigidBodyLink::init(stage);
      }
    }
    else if(stage==unknownStage) {
      if(body[0] and body[0]->getuRelSize()!=1)
        THROW_MBSIMERROR("rigid body on first side to must have of 1 dof!");
      if(body[1]->getuRelSize()!=1)
        THROW_MBSIMERROR("rigid body on second side must have 1 dof!");
      RigidBodyLink::init(stage);
    }
    else
      RigidBodyLink::init(stage);
    func->init(stage);
  }

  void GeneralizedSpringDamper::plot(double t,double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (coilspringOpenMBV) {
        Vec WrOToPoint;
        Vec WrOFromPoint;

        WrOFromPoint = body[0]?body[0]->getFrameForKinematics()->getPosition(t):body[1]->getFrameOfReference()->getPosition(t);
        WrOToPoint   = body[1]->getFrameForKinematics()->getPosition(t);
        vector<double> data;
        data.push_back(t);
        data.push_back(WrOFromPoint(0));
        data.push_back(WrOFromPoint(1));
        data.push_back(WrOFromPoint(2));
        data.push_back(WrOToPoint(0));
        data.push_back(WrOToPoint(1));
        data.push_back(WrOToPoint(2));
        data.push_back(fabs(getGeneralizedForce(t)(0)));
        coilspringOpenMBV->append(data);
      }
#endif
      RigidBodyLink::plot(t,dt);
    }
  }

  void GeneralizedSpringDamper::initializeUsingXML(DOMElement *element) {
    RigidBodyLink::initializeUsingXML(element);
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
#endif
  }

}
