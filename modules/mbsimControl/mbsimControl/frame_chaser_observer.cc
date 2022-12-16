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
#include "mbsimControl/frame_chaser_observer.h"
#include "mbsimControl/signal_.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, FrameChaserObserver)

  FrameChaserObserver::FrameChaserObserver(const std::string &name) : Observer(name) {
  }

  void FrameChaserObserver::init(InitStage stage, const MBSim::InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(!saved_frame.empty())           setFrame          (getByPath<MBSim::Frame>(saved_frame));
      if(!saved_signalX.empty())         setSignalX        (getByPath<Signal>      (saved_signalX));
      if(!saved_signalY.empty())         setSignalY        (getByPath<Signal>      (saved_signalY));
      if(!saved_signalZ.empty())         setSignalZ        (getByPath<Signal>      (saved_signalZ));
      if(!saved_signalRotationX.empty()) setSignalRotationX(getByPath<Signal>      (saved_signalRotationX));
      if(!saved_signalRotationY.empty()) setSignalRotationY(getByPath<Signal>      (saved_signalRotationY));
      if(!saved_signalRotationZ.empty()) setSignalRotationZ(getByPath<Signal>      (saved_signalRotationZ));
      Observer::init(stage, config);
    }
    else if(stage==plotting) {
      Observer::init(stage, config);
      if(plotFeature[MBSim::openMBV]) {
        if(ombvFrame) {
          openMBVFrame=ombvFrame->createOpenMBV();
          openMBVFrame->setName("FrameChaser");
          getOpenMBVGrp()->addObject(openMBVFrame);
        }
      }
    }
    else
      Observer::init(stage, config);
  }

  namespace {
    struct CompSetter {
      CompSetter(double &value_) : value(value_) {}
      void operator()(std::monostate) {}
      void operator()(double v) { value = v; }
      void operator()(Signal* sig) { value = sig->evalSignal()(0); }
      private:
        double &value;
    };
  }
  void FrameChaserObserver::plot() {
    if(plotFeature[MBSim::openMBV]) {
      if(openMBVFrame) {
        // translation and rotation is 0 per default
        Vec3 r(INIT, 0.0);
        Vec3 revCardan(INIT, 0.0);
        // if a frame is reference use its position/orientation
        if(frame) {
          r=frame->evalPosition();
          revCardan=MBSim::AIK2RevCardan(frame->evalOrientation());
        }
        // overwrite components if given
        std::visit(CompSetter(r(0))        , x);
        std::visit(CompSetter(r(1))        , y);
        std::visit(CompSetter(r(2))        , z);
        std::visit(CompSetter(revCardan(0)), rotX);
        std::visit(CompSetter(revCardan(1)), rotY);
        std::visit(CompSetter(revCardan(2)), rotZ);

        // convert revCardan to cardan and set openmbv frame data
        auto T2=MBSim::Cardan2AIK(-revCardan(0),-revCardan(1),-revCardan(2)).T();
        auto cardan=MBSim::AIK2Cardan(T2);
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

  void FrameChaserObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);

    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"frame");
    if(e) saved_frame=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"constantX");
    if(e) setConstantX(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"constantY");
    if(e) setConstantY(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"constantZ");
    if(e) setConstantZ(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"constantRotationX");
    if(e) setConstantRotationX(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"constantRotationY");
    if(e) setConstantRotationY(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"constantRotationZ");
    if(e) setConstantRotationZ(E(e)->getText<double>());

    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"signalX");
    if(e) saved_signalX=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"signalY");
    if(e) saved_signalY=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"signalZ");
    if(e) saved_signalZ=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"signalRotationX");
    if(e) saved_signalRotationX=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"signalRotationY");
    if(e) saved_signalRotationY=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"signalRotationZ");
    if(e) saved_signalRotationZ=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"enableOpenMBV");
    if(e) {
      ombvFrame = make_shared<MBSim::OpenMBVFrame>();
      ombvFrame->initializeUsingXML(e);
    }
  }

}
