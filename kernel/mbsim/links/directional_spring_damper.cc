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
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/coilspring.h>
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/objectfactory.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(DirectionalSpringDamper, MBSIM%"DirectionalSpringDamper")

  DirectionalSpringDamper::DirectionalSpringDamper(const string &name) : FloatingFrameLink(name), func(NULL), l0(0) {
  }

  DirectionalSpringDamper::~DirectionalSpringDamper() {
    delete func;
  }

  void DirectionalSpringDamper::updatePositions(double t, Frame *frame_) {
    frame_->setPosition(frame[1]->getPosition() - getGlobalForceDirection(t)*(getGlobalForceDirection(t).T()*getGlobalRelativePosition(t)));
    frame_->setOrientation(frame[0]->getOrientation());
  }

  void DirectionalSpringDamper::updateGeneralizedPositions(double t) {
    rrel=getGlobalForceDirection(t).T()*getGlobalRelativePosition(t);
    updrrel = false;
  }

  void DirectionalSpringDamper::updateGeneralizedVelocities(double t) {
    vrel=getGlobalForceDirection(t).T()*getGlobalRelativeVelocity(t);
    updvrel = false;
  }

  void DirectionalSpringDamper::updatelaF(double t) {
    lambdaF(0)=-(*func)(evalGeneralizedRelativePosition()(0)-l0,evalGeneralizedRelativeVelocity()(0));
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

          WrOFromPoint = C.evalPosition();
          WrOToPoint   = frame[1]->evalPosition();
          vector<double> data;
          data.push_back(t); 
          data.push_back(WrOFromPoint(0));
          data.push_back(WrOFromPoint(1));
          data.push_back(WrOFromPoint(2));
          data.push_back(WrOToPoint(0));
          data.push_back(WrOToPoint(1));
          data.push_back(WrOToPoint(2));
          data.push_back(fabs(evalGeneralizedForce()(0)));
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

}
