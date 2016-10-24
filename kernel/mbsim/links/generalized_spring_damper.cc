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
#include "mbsim/links/generalized_spring_damper.h"
#include "mbsim/objectfactory.h"
#include "mbsim/objects/rigid_body.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/coilspring.h>
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/objectfactory.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedSpringDamper, MBSIM%"GeneralizedSpringDamper")

  GeneralizedSpringDamper::GeneralizedSpringDamper(const string &name) : RigidBodyLink(name), func(NULL), l0(0) {
    body[0] = NULL;
    body[1] = NULL;
  }

  GeneralizedSpringDamper::~GeneralizedSpringDamper() {
    delete func;
  }

  void GeneralizedSpringDamper::updateGeneralizedForces() {
    for(int i=0; i<lambda.size(); i++)
      lambda(i) = -(*func)(evalGeneralizedRelativePosition()(i)-l0,evalGeneralizedRelativeVelocity()(i));
    updla = false;
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
    else
      RigidBodyLink::init(stage);
    func->init(stage);
  }

  void GeneralizedSpringDamper::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (coilspringOpenMBV) {
        Vec WrOToPoint;
        Vec WrOFromPoint;

        WrOFromPoint = body[0]?body[0]->getFrameForKinematics()->evalPosition():body[1]->getFrameOfReference()->evalPosition();
        WrOToPoint   = body[1]->getFrameForKinematics()->evalPosition();
        vector<double> data;
        data.push_back(getTime());
        data.push_back(WrOFromPoint(0));
        data.push_back(WrOFromPoint(1));
        data.push_back(WrOFromPoint(2));
        data.push_back(WrOToPoint(0));
        data.push_back(WrOToPoint(1));
        data.push_back(WrOToPoint(2));
        data.push_back(fabs(evalGeneralizedForce()(0)));
        coilspringOpenMBV->append(data);
      }
#endif
      RigidBodyLink::plot();
    }
  }

  void GeneralizedSpringDamper::initializeUsingXML(DOMElement *element) {
    RigidBodyLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedForceFunction");
    Function<double(double,double)> *f=ObjectFactory::createAndInit<Function<double(double,double)> >(e->getFirstElementChild());
    setGeneralizedForceFunction(f);
    e = E(element)->getFirstElementChildNamed(MBSIM%"unloadedGeneralizedLength");
    l0 = Element::getDouble(e);
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
