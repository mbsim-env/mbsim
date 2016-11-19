/* Copyright (C) 2004-2016 MBSim Development Team
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
#include "mbsim/links/generalized_elastic_connection.h"
#include "mbsim/objectfactory.h"
#include "mbsim/objects/rigid_body.h"
#include <openmbvcppinterface/coilspring.h>
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/objectfactory.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, GeneralizedElasticConnection)

  GeneralizedElasticConnection::GeneralizedElasticConnection(const string &name) : RigidBodyLink(name), func(NULL) {
    body[0] = NULL;
    body[1] = NULL;
  }

  GeneralizedElasticConnection::~GeneralizedElasticConnection() {
    delete func;
  }

  void GeneralizedElasticConnection::updateGeneralizedForces() {
    lambda = -(*func)(evalGeneralizedRelativePosition(),evalGeneralizedRelativeVelocity());
    updla = false;
  }

  void GeneralizedElasticConnection::init(InitStage stage) {
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
    else if(stage==unknownStage) {
      if(body[0] and body[0]->getuRelSize()!=body[1]->getuRelSize())
        THROW_MBSIMERROR("rigid bodies must have the same dof!");
      RigidBodyLink::init(stage);
    }
    else
      RigidBodyLink::init(stage);
    func->init(stage);
  }

  void GeneralizedElasticConnection::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      RigidBodyLink::plot();
    }
  }

  void GeneralizedElasticConnection::initializeUsingXML(DOMElement *element) {
    RigidBodyLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedForceFunction");
    Function<VecV(VecV,VecV)> *f=ObjectFactory::createAndInit<Function<VecV(VecV,VecV)> >(e->getFirstElementChild());
    setGeneralizedForceFunction(f);
    e=E(element)->getFirstElementChildNamed(MBSIM%"rigidBodyFirstSide");
    if(e) saved_body1=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"rigidBodySecondSide");
    saved_body2=E(e)->getAttribute("ref");
  }

}
