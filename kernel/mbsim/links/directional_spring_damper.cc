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
#include "mbsim/links/directional_spring_damper.h"
#include "mbsim/objectfactory.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include <openmbvcppinterface/coilspring.h>
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/objectfactory.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, DirectionalSpringDamper)

  DirectionalSpringDamper::DirectionalSpringDamper(const string &name) : FloatingFrameLink(name), func(nullptr) {
    evalOMBVColorRepresentation[0] = &DirectionalSpringDamper::evalNone;
    evalOMBVColorRepresentation[1] = &DirectionalSpringDamper::evalDeflection;
    evalOMBVColorRepresentation[2] = &DirectionalSpringDamper::evalTensileForce;
    evalOMBVColorRepresentation[3] = &DirectionalSpringDamper::evalCompressiveForce;
    evalOMBVColorRepresentation[4] = &DirectionalSpringDamper::evalAbsoluteForce;
  }

  DirectionalSpringDamper::~DirectionalSpringDamper() {
    delete func;
  }

  void DirectionalSpringDamper::updatePositions(Frame *frame_) {
    frame_->setPosition(frame[1]->evalPosition() - evalGlobalForceDirection()*(evalGlobalForceDirection().T()*evalGlobalRelativePosition()));
    frame_->setOrientation(frame[0]->getOrientation());
  }

  void DirectionalSpringDamper::updateGeneralizedPositions() {
    rrel=evalGlobalForceDirection().T()*evalGlobalRelativePosition();
    updrrel = false;
  }

  void DirectionalSpringDamper::updateGeneralizedVelocities() {
    vrel=evalGlobalForceDirection().T()*evalGlobalRelativeVelocity();
    updvrel = false;
  }

  void DirectionalSpringDamper::updatelaF() {
    lambdaF(0)=-(*func)(evalGeneralizedRelativePosition()(0)-l0,evalGeneralizedRelativeVelocity()(0));
    updlaF = false;
  }

  void DirectionalSpringDamper::init(InitStage stage, const InitConfigSet &config) {
    if(stage==plotting) {
      if(plotFeature[openMBV] and ombvCoilSpring) {
        coilspringOpenMBV=ombvCoilSpring->createOpenMBV();
        coilspringOpenMBV->setName(name);
        parent->getLinksOpenMBVGrp()->addObject(coilspringOpenMBV);
      }
    }
    FloatingFrameLink::init(stage, config);
    func->init(stage, config);
  }

  void DirectionalSpringDamper::plot() {
    if(plotFeature[openMBV]) {
      if (coilspringOpenMBV) {
        Vec3 WrOToPoint;
        Vec3 WrOFromPoint;

        WrOFromPoint = C.evalPosition();
        WrOToPoint   = frame[1]->evalPosition();
        array<double,8> data;
        data[0] = getTime();
        data[1] = WrOFromPoint(0);
        data[2] = WrOFromPoint(1);
        data[3] = WrOFromPoint(2);
        data[4] = WrOToPoint(0);
        data[5] = WrOToPoint(1);
        data[6] = WrOToPoint(2);
        data[7] = (this->*evalOMBVColorRepresentation[ombvCoilSpring->getColorRepresentation()])();
        coilspringOpenMBV->append(data);
      }
    }
    FloatingFrameLink::plot();
  }

  void DirectionalSpringDamper::initializeUsingXML(DOMElement *element) {
    FloatingFrameLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"forceDirection");
    setForceDirection(E(e)->getText<Vec>(3));
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
    auto *f=ObjectFactory::createAndInit<Function<double(double,double)>>(e->getFirstElementChild());
    setForceFunction(f);
    e=E(element)->getFirstElementChildNamed(MBSIM%"unloadedLength");
    l0 = E(e)->getText<double>();
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      ombvCoilSpring = shared_ptr<OpenMBVCoilSpring>(new OpenMBVCoilSpring);
      ombvCoilSpring->initializeUsingXML(e);
    }
  }

}
