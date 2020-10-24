/* Copyright (C) 2004-2017 MBSim Development Team
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
#include "mbsim/observers/filtered_frame_observer.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, FilteredFrameObserver)

  FilteredFrameObserver::FilteredFrameObserver(const std::string &name) : Observer(name), frame(nullptr) {
  }

  void FilteredFrameObserver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not saved_frame.empty())
        setFrame(getByPath<Frame>(saved_frame));
      if(not frame)
        throwError("Frame is not given!");
      Observer::init(stage, config);
    }
    else if(stage==plotting) {
      Observer::init(stage, config);
      if(plotFeature[openMBV]) {
        if(ombvFrame) {
          openMBVFrame=ombvFrame->createOpenMBV();
          openMBVFrame->setName("FilteredFrame");
          getOpenMBVGrp()->addObject(openMBVFrame);
        }
      }
    }
    else
      Observer::init(stage, config);
  }

  void FilteredFrameObserver::plot() {
    if(plotFeature[openMBV]) {
      if(openMBVFrame && !openMBVFrame->isHDF5Link()) {
        auto r=frame->evalPosition();
        if(constX) r(0)=*constX;
        if(constY) r(1)=*constY;
        if(constZ) r(2)=*constZ;

        auto& T=frame->evalOrientation();
        auto revCardan=AIK2RevCardan(T);
        if(constRotX) revCardan(0)=*constRotX;
        if(constRotY) revCardan(1)=*constRotY;
        if(constRotZ) revCardan(2)=*constRotZ;
        auto T2=Cardan2AIK(-revCardan(0),-revCardan(1),-revCardan(2)).T();
        auto cardan=AIK2Cardan(T2);

        vector<double> data;
        data.push_back(getTime());
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        data.push_back(cardan(0));
        data.push_back(cardan(1));
        data.push_back(cardan(2));
        data.push_back(0);
        openMBVFrame->append(data);
      }
    }
    Observer::plot();
  }

  void FilteredFrameObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"frame");
    saved_frame=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"constantX");
    if(e) setConstantX(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"constantY");
    if(e) setConstantY(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"constantZ");
    if(e) setConstantZ(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"constantRotationX");
    if(e) setConstantRotationX(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"constantRotationY");
    if(e) setConstantRotationY(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"constantRotationZ");
    if(e) setConstantRotationZ(E(e)->getText<double>());

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      ombvFrame = shared_ptr<OpenMBVFrame>(new OpenMBVFrame);
      ombvFrame->initializeUsingXML(e);
    }
  }

}
