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
#include "aerodynamics.h"
#include "mbsimPhysics/namespace.h"
#include "mbsim/dynamic_system.h"
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

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMPHYSICS, Aerodynamics)

  Aerodynamics::Aerodynamics(const std::string &name) : FloatingFrameLink(name) {
    forceDir.resize(3,EYE);
  }

  Aerodynamics::~Aerodynamics() {
    delete frho;
    delete fc;
  }

  void Aerodynamics::setDensityFunction(MBSim::Function<double(double)> *func) {
    frho=func;
    frho->setParent(this);
    frho->setName("Density");
  }

  void Aerodynamics::setCoefficientFunction(MBSim::Function<Vec3(Vec2)> *func) {
    fc=func;
    fc->setParent(this);
    fc->setName("Coefficient");
  }

  void Aerodynamics::updatelaF() {
    Vec3 vE = evalGlobalRelativeVelocity() - frame[0]->evalOrientation()*vW;
    double absvE = nrm2(vE);
    Vec3 KvE = frame[1]->evalOrientation().T()*vE;
    Vec2 bega(NONINIT);
    if(absvE>1e-13)
      bega(1) = asin(max(min(KvE(1)/absvE,1.0),-1.0));
    else
      bega(1) = 0;
    bega(0) = atan2(-KvE(2),KvE(0));
    Vec3 c = (*fc)(bega);
    Vec3 F(NONINIT);
    F(0) = -cos(bega(0))*cos(bega(1))*c(0) - cos(bega(0))*sin(bega(1))*c(1) + sin(bega(0))*c(2);
    F(1) = -sin(bega(1))*c(0) + cos(bega(1))*c(1);
    F(2) = sin(bega(0))*cos(bega(1))*c(0) + sin(bega(0))*sin(bega(1))*c(1) + cos(bega(0))*c(2);
    double h = frame[0]->getOrientation().col(1).T()*evalGlobalRelativePosition();
    lambdaF = 0.5*(*frho)(h)*A*pow(absvE,2)*(frame[1]->getOrientation()*F);
    updlaF = false;
  }

  void Aerodynamics::init(InitStage stage, const MBSim::InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not frame[0]) frame[0] = static_cast<DynamicSystem*>(parent)->getFrameI();
//      if(not frho) setDensityFunction(new ConstantFunction<double(double)>(1.2041));
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
    FloatingFrameLink::init(stage,config);
    frho->init(stage, config);
    fc->init(stage, config);
  }

  void Aerodynamics::plot() {
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

  void Aerodynamics::initializeUsingXML(xercesc::DOMElement *element) {
    FloatingFrameLink::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"densityFunction");
    setDensityFunction(ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
    e = E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"coefficientFunction");
    setCoefficientFunction(ObjectFactory::createAndInit<MBSim::Function<Vec3(Vec2)> >(e->getFirstElementChild()));
    e = E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"referenceSurface");
    if(e) setReferenceSurface(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"windSpeed");
    if(e) setWindSpeed(E(e)->getText<Vec3>());
    e=E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"enableOpenMBV");
    if(e) {
      ombvArrow = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvArrow->initializeUsingXML(e);
    }
  }

}
