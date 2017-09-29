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
#include "mbsim/utils/eps.h"
#include "mbsim/objectfactory.h"
#include <openmbvcppinterface/coilspring.h>
#include "openmbvcppinterface/group.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, SpringDamper)

  const std::size_t SpringDamper::deflection = std::hash<std::string>()("deflection");

  SpringDamper::SpringDamper(const string &name) : FixedFrameLink(name), func(NULL), l0(0) {
  }

  SpringDamper::~SpringDamper() {
    delete func;
  }

  void SpringDamper::updatelaF() {
    lambdaF(0)=-(*func)(evalGeneralizedRelativePosition()(0)-l0,evalGeneralizedRelativeVelocity()(0));
    if(rrel(0)<=epsroot() && abs(lambda(0))>epsroot())
      msg(Warn)<<"The SpringDamper force is not 0 and the force direction can not calculated!\nUsing force=0 at t="<<getTime()<<endl;
    updlaF = false;
  }

  void SpringDamper::init(InitStage stage) {
    if(stage==plotting) {
      if(plotFeature[plotRecursive]==enabled and plotFeature[deflection]==enabled)
          plotColumns.push_back("deflection");
      if(plotFeature[openMBV]==enabled and coilspringOpenMBV) {
        coilspringOpenMBV->setName(name);
        parent->getOpenMBVGrp()->addObject(coilspringOpenMBV);
      }
    }
    FixedFrameLink::init(stage);
    func->init(stage);
  }

  void SpringDamper::plot() {
    if(plotFeature[plotRecursive]==enabled and plotFeature[deflection]==enabled)
      plotVector.push_back(evalGeneralizedRelativePosition()(0)-l0);
    if(plotFeature[openMBV]==enabled and coilspringOpenMBV) {
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
      data.push_back(fabs(evalGeneralizedForce()(0)));
      coilspringOpenMBV->append(data);
    }
    FixedFrameLink::plot();
  }

  void SpringDamper::initializeUsingXML(DOMElement *element) {
    FixedFrameLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
    Function<double(double,double)> *f=ObjectFactory::createAndInit<Function<double(double,double)> >(e->getFirstElementChild());
    setForceFunction(f);
    e = E(element)->getFirstElementChildNamed(MBSIM%"unloadedLength");
    l0 = Element::getDouble(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVCoilSpring ombv;
      coilspringOpenMBV=ombv.createOpenMBV(e);
    }
  }

}
