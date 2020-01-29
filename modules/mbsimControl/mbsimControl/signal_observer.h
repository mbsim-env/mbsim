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

#ifndef _SIGNAL_OBSERVER_H__
#define _SIGNAL_OBSERVER_H__

#include "mbsim/observers/observer.h"

#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>

namespace MBSimControl {

  class Signal;

  class SignalObserver : public MBSim::Observer {
    private:
      Signal *signal{nullptr};
      Signal *position{nullptr};
      std::shared_ptr<MBSim::OpenMBVArrow> ombvArrow;
      std::shared_ptr<OpenMBV::Arrow> openMBVArrow;
      std::string saved_signal;
      std::string saved_position;

    public:
      SignalObserver(const std::string &name="") : MBSim::Observer(name) { }

      void setSignal(Signal *signal_) { signal = signal_; }
      void setPosition(Signal *position_) { position = position_; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void plot() override;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(MBSim::OpenMBVArrow::ReferencePoint),MBSim::OpenMBVArrow::fromPoint)(colorRepresentation,(MBSim::OpenMBVArrow::ColorRepresentation),MBSim::OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvArrow = std::shared_ptr<MBSim::OpenMBVArrow>(new MBSim::OpenMBVArrow(scaleLength,scaleSize,MBSim::OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }
  };

}

#endif
