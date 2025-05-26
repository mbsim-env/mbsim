/* Copyright (C) 2004-2018 MBSim Development Team
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
#include "drag.h"
#include "mbsimPhysics/namespace.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/functions/function.h"
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/arrow.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimPhysics {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMPHYSICS, Drag)

  Drag::Drag(const std::string &name) : FloatingFrameLink(name) {
    forceDir.resize(3,EYE);
  }

  Drag::~Drag() {
    delete fdrag;
  }

  void Drag::setDragFunction(MBSim::Function<double(double)> *func) {
    fdrag=func;
    fdrag->setParent(this);
    fdrag->setName("Drag");
  }

  void Drag::updatelaF() {
    Vec3 v = evalGlobalRelativeVelocity();
    double vnrm = nrm2(v);
    if(vnrm<=1e-13)
      lambdaF.init(0);
    else
      lambdaF = (-(*fdrag)(vnrm)/vnrm)*(frame[refFrame]->getOrientation().T()*v);
    updlaF = false;
  }

  void Drag::init(InitStage stage, const MBSim::InitConfigSet &config) {
    if(stage==plotting) {
      if(plotFeature[openMBV] and ombvArrow) {
        openMBVForce.resize(ombvArrow->getSideOfInteraction()==2?getNumberOfForces():getNumberOfForces()/2);
        for(size_t i=0; i<openMBVForce.size(); i++) {
          openMBVForce[i]=ombvArrow->createOpenMBV();
          openMBVForce[i]->setName(name+"_Force"+(openMBVForce.size()>1?to_string(i):string("")));
          parent->getOpenMBVGrp()->addObject(openMBVForce[i]);
        }
      }
    }
    fdrag->init(stage,config);
    FloatingFrameLink::init(stage,config);
  }

  void Drag::plot() {
    if(plotFeature[openMBV] and ombvArrow) {
      int off = ombvArrow->getSideOfInteraction()==0?getNumberOfForces()/2:0;
      for(size_t i=0; i<openMBVForce.size(); i++) {
        array<double,8> data;
        data[0] = getTime();
        Vec3 toPoint=getPointOfApplication(off+i)->evalPosition();
        data[1] = toPoint(0);
        data[2] = toPoint(1);
        data[3] = toPoint(2);
        Vec3 WF = evalForce(off+i);
        data[4] = WF(0);
        data[5] = WF(1);
        data[6] = WF(2);
        data[7] = ombvArrow->getColorRepresentation()?nrm2(evalForce()):1;
        openMBVForce[i]->append(data);
      }
    }
    MechanicalLink::plot();
  }

  void Drag::initializeUsingXML(xercesc::DOMElement *element) {
    FloatingFrameLink::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"dragFunction");
    setDragFunction(ObjectFactory::createAndInit<MBSim::Function<double(double)>>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"enableOpenMBV");
    if(e) {
      ombvArrow = make_shared<OpenMBVInteractionArrow>(0,1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint);
      ombvArrow->initializeUsingXML(e);
    }
  }

}
