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
#include "mbsim/links/spring_damper.h"
#include "mbsim/frames/frame.h"
#include "mbsim/objectfactory.h"
#include <openmbvcppinterface/coilspring.h>
#include "openmbvcppinterface/group.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, SpringDamper)

  const PlotFeatureEnum deflection;

  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, deflection)

  SpringDamper::SpringDamper(const string &name) : FixedFrameLink(name), func(NULL), l0(0) {
    nF = 1;
    evalOMBVColorRepresentation[0] = &SpringDamper::evalNone;
    evalOMBVColorRepresentation[1] = &SpringDamper::evalDeflection;
    evalOMBVColorRepresentation[2] = &SpringDamper::evalTensileForce;
    evalOMBVColorRepresentation[3] = &SpringDamper::evalCompressiveForce;
    evalOMBVColorRepresentation[4] = &SpringDamper::evalAbsoluteForce;
  }

  SpringDamper::~SpringDamper() {
    delete func;
  }

  void SpringDamper::updatelaF() {
    lambdaF(0)=-(*func)(evalGeneralizedRelativePosition()(0)-l0,evalGeneralizedRelativeVelocity()(0));
    if(getGeneralizedRelativePosition()(0)<=1e-13 && abs(lambdaF(0))>1e-13)
      msg(Warn)<<"The SpringDamper force is not 0 and the force direction can not calculated!\nUsing force=0 at t="<<getTime()<<endl;
    updlaF = false;
  }

  void SpringDamper::init(InitStage stage, const InitConfigSet &config) {
    if(stage==plotting) {
      if(plotFeature[plotRecursive] and plotFeature[MBSim::deflection])
          plotColumns.push_back("deflection");
      if(plotFeature[openMBV] and ombvCoilSpring) {
        coilspringOpenMBV=ombvCoilSpring->createOpenMBV();
        coilspringOpenMBV->setName(name);
        parent->getLinksOpenMBVGrp()->addObject(coilspringOpenMBV);
      }
    }
    FixedFrameLink::init(stage, config);
    func->init(stage, config);
  }

  void SpringDamper::plot() {
    if(plotFeature[plotRecursive] and plotFeature[MBSim::deflection])
      plotVector.push_back(evalGeneralizedRelativePosition()(0)-l0);
    if(plotFeature[openMBV] and coilspringOpenMBV) {
      Vec3 WrOToPoint;
      Vec3 WrOFromPoint;

      WrOFromPoint = frame[0]->evalPosition();
      WrOToPoint   = frame[1]->evalPosition();
      vector<double> data;
      data.push_back(getTime());
      data.push_back(WrOFromPoint(0));
      data.push_back(WrOFromPoint(1));
      data.push_back(WrOFromPoint(2));
      data.push_back(WrOToPoint(0));
      data.push_back(WrOToPoint(1));
      data.push_back(WrOToPoint(2));
      data.push_back((this->*evalOMBVColorRepresentation[ombvCoilSpring->getColorRepresentation()])());
      coilspringOpenMBV->append(data);
    }
    FixedFrameLink::plot();
  }

  void SpringDamper::initializeUsingXML(DOMElement *element) {
    FixedFrameLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
    Function<double(double,double)> *f=ObjectFactory::createAndInit<Function<double(double,double)>>(e->getFirstElementChild());
    setForceFunction(f);
    e = E(element)->getFirstElementChildNamed(MBSIM%"unloadedLength");
    l0 = E(e)->getText<double>();
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      ombvCoilSpring = shared_ptr<OpenMBVCoilSpring>(new OpenMBVCoilSpring);
      ombvCoilSpring->initializeUsingXML(e);
    }
  }

}
