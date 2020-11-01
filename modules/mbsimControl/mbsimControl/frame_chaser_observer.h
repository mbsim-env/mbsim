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

#ifndef _MBSIMCONTROl_FRAME_CHASER_OBSERVER_H__
#define _MBSIMCONTROl_FRAME_CHASER_OBSERVER_H__

#include "mbsim/observers/observer.h"

#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>
#include <variant>

namespace MBSim {
  class Frame;
}

namespace MBSimControl {
  
  class Signal;

  class FrameChaserObserver : public MBSim::Observer {
    protected:
      MBSim::Frame* frame { nullptr };

      std::string saved_frame;
      std::string saved_signalX, saved_signalY, saved_signalZ;
      std::string saved_signalRotationX, saved_signalRotationY, saved_signalRotationZ;

      std::shared_ptr<MBSim::OpenMBVFrame> ombvFrame;
      std::shared_ptr<OpenMBV::Frame> openMBVFrame;

      // overrite a component of the frame observer with a double or value from a Signal
      std::variant<std::monostate, double, Signal*> x;
      std::variant<std::monostate, double, Signal*> y;
      std::variant<std::monostate, double, Signal*> z;
      std::variant<std::monostate, double, Signal*> rotX;
      std::variant<std::monostate, double, Signal*> rotY;
      std::variant<std::monostate, double, Signal*> rotZ;

    public:
      FrameChaserObserver(const std::string &name="");
      void setFrame(MBSim::Frame *frame_) { frame = frame_; } 

      void setConstantX(double x_) { x=x_; }
      void setConstantY(double y_) { y=y_; }
      void setConstantZ(double z_) { z=z_; }
      void setConstantRotationX(double angle) { rotX=angle; }
      void setConstantRotationY(double angle) { rotY=angle; }
      void setConstantRotationZ(double angle) { rotZ=angle; }
      void setSignalX(Signal* x_) { x=x_; }
      void setSignalY(Signal* y_) { y=y_; }
      void setSignalZ(Signal* z_) { z=z_; }
      void setSignalRotationX(Signal* angle) { rotX=angle; }
      void setSignalRotationY(Signal* angle) { rotY=angle; }
      void setSignalRotationZ(Signal* angle) { rotZ=angle; }
      void unsetX() { x=std::monostate(); }
      void unsetY() { y=std::monostate(); }
      void unsetZ() { z=std::monostate(); }
      void unsetRotationX() { x=std::monostate(); }
      void unsetRotationY() { y=std::monostate(); }
      void unsetRotationZ() { z=std::monostate(); }

      void init(InitStage stage, const MBSim::InitConfigSet &config);
      void plot();
      void initializeUsingXML(xercesc::DOMElement *element);

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (size,(double),1)(offset,(double),1)
        (transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvFrame = std::shared_ptr<MBSim::OpenMBVFrame>(new MBSim::OpenMBVFrame(size, offset, fmatvec::Vec3({-1,1,1}),
                                                                   transparency, pointSize, lineWidth));
      }
  };

}  

#endif
