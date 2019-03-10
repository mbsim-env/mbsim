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
#include "buoyancy.h"
#include "mbsimPhysics/namespace.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/functions/function.h"
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimPhysics {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMPHYSICS, Buoyancy)

  Buoyancy::Buoyancy(const std::string &name) : FloatingFrameLink(name) {
    forceDir = Mat3xV(1);
    forceDir(1,0) = 1;
  }

  Buoyancy::~Buoyancy() {
    delete frho;
    delete fg;
  }

  void Buoyancy::setDensityFunction(MBSim::Function<double(double)> *func) {
    frho=func;
    frho->setParent(this);
    frho->setName("Density");
  }

  void Buoyancy::setGravityFunction(MBSim::Function<double(double)> *func) {
    fg=func;
    fg->setParent(this);
    fg->setName("Gravity");
  }

  void Buoyancy::updatelaF() {
    lambdaF(0)=(*frho)(frame[0]->evalOrientation().col(1).T()*evalGlobalRelativePosition())*V*(*fg)(evalGeneralizedRelativePosition()(0));
    updlaF = false;
  }

  void Buoyancy::init(InitStage stage, const MBSim::InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not frame[0]) frame[0] = static_cast<DynamicSystem*>(parent)->getFrameI();
//      if(not frho) setDensityFunction(new ConstantFunction<double(double)>(1.2041));
//      if(not fg) setGravityFunction(new ConstantFunction<double(double)>(9.80665));
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] and ombvArrow) {
        openMBVForce.resize(ombvArrow->getSideOfInteraction()==2?getNumberOfLinks():getNumberOfLinks()/2);
        for(size_t i=0; i<openMBVForce.size(); i++) {
          openMBVForce[i]=ombvArrow->createOpenMBV();
          openMBVForce[i]->setName(name+"_Force"+(openMBVForce.size()>1?to_string(i):string("")));
          parent->getOpenMBVGrp()->addObject(openMBVForce[i]);
        }
      }
    }
    frho->init(stage,config);
    fg->init(stage,config);
    FloatingFrameLink::init(stage,config);
  }

  void Buoyancy::plot() {
    if(plotFeature[openMBV] and ombvArrow) {
      int off = ombvArrow->getSideOfInteraction()==0?getNumberOfLinks()/2:0;
      for(size_t i=0; i<openMBVForce.size(); i++) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 toPoint=getPointOfApplication(off+i)->evalPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 WF = evalForce(off+i);
        data.push_back(WF(0));
        data.push_back(WF(1));
        data.push_back(WF(2));
        data.push_back(ombvArrow->getColorRepresentation()?nrm2(evalForce()):1);
        openMBVForce[i]->append(data);
      }
    }
    MechanicalLink::plot();
  }

  void Buoyancy::initializeUsingXML(xercesc::DOMElement *element) {
    FloatingFrameLink::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"displacedVolume");
    setDisplacedVolume(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"densityFunction");
    setDensityFunction(ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
    e = E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"gravityFunction");
    setGravityFunction(ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"enableOpenMBV");
    if(e) {
      ombvArrow = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvArrow->initializeUsingXML(e);
    }
  }

}
