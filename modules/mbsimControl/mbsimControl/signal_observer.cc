/* Copyright (C) 2004-2020 MBSim Development Team
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
#include "mbsimControl/signal_observer.h"
#include "mbsimControl/signal_.h"
#include "mbsimControl/extern_signal_source.h"
#include "mbsim/frames/frame.h"
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/ivscreenannotation.h>

using namespace std;
using namespace xercesc;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, SignalObserver)

  void SignalObserver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not saved_signal.empty())
        setSignal(getByPath<Signal>(saved_signal));
      if(not signal)
        throwError("Signal is not given!");
      if(not saved_position.empty())
        setPosition(getByPath<Signal>(saved_position));
      Observer::init(stage, config);
    }
    else if(stage==plotting) {
      if(plotFeature[plotRecursive]) {
        if(plotFeature[MBSimControl::signal])
          addToPlot("signal",signal->getSignalSize());
      }
      Observer::init(stage, config);
      if(plotFeature[openMBV]) {
        if(ombvArrow) {
          openMBVArrow=ombvArrow->createOpenMBV();
          openMBVArrow->setName("Signal");
          getOpenMBVGrp()->addObject(openMBVArrow);
        }
        if(openMBVIvScreenAnnotation) {
          openMBVIvScreenAnnotation->setName("SignalVisualization");
          getOpenMBVGrp()->addObject(openMBVIvScreenAnnotation);
        }
      }
    }
    else
      Observer::init(stage, config);
  }

  void SignalObserver::plot() {
    auto s = signal->evalSignal();
    if(plotFeature[plotRecursive]) {
      if(plotFeature[MBSimControl::signal])
	Element::plot(s);
    }
    if(plotFeature[openMBV]) {
      Vec3 r(INIT, 0.0);
      if(position) r = position->evalSignal();
      if(openMBVArrow) {
        vector<double> data;
        data.push_back(getTime());
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        data.push_back(s(0));
        data.push_back(s(1));
        data.push_back(s(2));
        data.push_back(ombvArrow->getColorRepresentation()?nrm2(s):0.5);
        openMBVArrow->append(data);
      }
      if(openMBVIvScreenAnnotation)
        openMBVIvScreenAnnotation->append(s);
    }
    Observer::plot();
  }

  void SignalObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"signal");
    saved_signal=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"position");
    if(e) saved_position=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"enableOpenMBV");
    if(e) {
      ombvArrow = make_shared<OpenMBVArrow>();
      ombvArrow->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"openMBVIvScreenAnnotation");
    if(e) {
      auto *ee=e->getFirstElementChild();
      auto object=OpenMBV::ObjectFactory::create<OpenMBV::IvScreenAnnotation>(ee);
      setOpenMBVIvScreenAnnotation(object);
      object->initializeUsingXML(ee);
    }
  }

  void SignalObserver::setOpenMBVIvScreenAnnotation(const std::shared_ptr<OpenMBV::IvScreenAnnotation> &object) {
    openMBVIvScreenAnnotation = object;
  }

  shared_ptr<OpenMBV::IvScreenAnnotation> SignalObserver::getOpenMBVIvScreenAnnotation() const {
    return openMBVIvScreenAnnotation;
  }

}
