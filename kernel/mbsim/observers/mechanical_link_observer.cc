/* Copyright (C) 2004-2016 MBSim Development Team
 *
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
 * Contact: martin.o.foerg@gmail.com
 */

#include <config.h>
#include "mbsim/observers/mechanical_link_observer.h"
#include "mbsim/links/mechanical_link.h"
#include "mbsim/frames/frame.h"
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, MechanicalLinkObserver)

  MechanicalLinkObserver::MechanicalLinkObserver(const std::string &name) : Observer(name), link(NULL) {
  }

  void MechanicalLinkObserver::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_link!="")
        setMechanicalLink(getByPath<MechanicalLink>(saved_link));
      Observer::init(stage);
    }
    else if(stage==plotting) {
      Observer::init(stage);
      if(plotFeature[openMBV]) {
//      if(openMBVForce) plotColumns.push_back("Force");
//      if(openMBVMoment) plotColumns.push_back("Moment");
        if(openMBVForce) {
          openMBVForce->setName("Force");
          getOpenMBVGrp()->addObject(openMBVForce);
        }
        if(openMBVMoment) {
          openMBVMoment->setName("Moment");
          getOpenMBVGrp()->addObject(openMBVMoment);
        }
      }
    }
    else
      Observer::init(stage);
  }

  void MechanicalLinkObserver::plot() {
    if(plotFeature[openMBV]) {
      if(openMBVForce) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 toPoint=link->getPointOfApplication(link->getNumberOfLinks()-1)->evalPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 WF = link->evalForce(link->getNumberOfLinks()-1);
        data.push_back(WF(0));
        data.push_back(WF(1));
        data.push_back(WF(2));
        data.push_back(nrm2(WF));
        openMBVForce->append(data);
      }
      if(openMBVMoment) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 toPoint=link->getPointOfApplication(link->getNumberOfLinks()-1)->evalPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 WM = link->evalMoment(link->getNumberOfLinks()-1);
        data.push_back(WM(0));
        data.push_back(WM(1));
        data.push_back(WM(2));
        data.push_back(nrm2(WM));
        openMBVMoment->append(data);
      }
    }
    Observer::plot();
  }

  void MechanicalLinkObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"mechanicalLink");
    saved_link=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(e) {
        OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
        openMBVForce=ombv.createOpenMBV(e); 
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(e) {
        OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
        openMBVMoment=ombv.createOpenMBV(e); 
    }
  }

}
