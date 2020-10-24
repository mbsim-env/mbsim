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

#ifndef _FILTERED_FRAME_OBSERVER_H__
#define _FILTERED_FRAME_OBSERVER_H__

#include "mbsim/observers/observer.h"

#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  class Frame;

  class FilteredFrameObserver : public Observer {
    protected:
      Frame* frame;
      std::string saved_frame;
      std::shared_ptr<OpenMBVFrame> ombvFrame;
      std::shared_ptr<OpenMBV::Frame> openMBVFrame;
      std::optional<double> constX;
      std::optional<double> constY;
      std::optional<double> constZ;
      std::optional<double> constRotX;
      std::optional<double> constRotY;
      std::optional<double> constRotZ;

    public:
      FilteredFrameObserver(const std::string &name="");
      void setFrame(Frame *frame_) { frame = frame_; } 

      void setConstantX(const std::optional<double>& x) { constX=x; }
      void setConstantY(const std::optional<double>& y) { constY=y; }
      void setConstantZ(const std::optional<double>& z) { constZ=z; }
      void setConstantRotationX(const std::optional<double>& angle) { constRotX=angle; }
      void setConstantRotationY(const std::optional<double>& angle) { constRotY=angle; }
      void setConstantRotationZ(const std::optional<double>& angle) { constRotZ=angle; }

      void init(InitStage stage, const InitConfigSet &config);
      void plot();
      void initializeUsingXML(xercesc::DOMElement *element);

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (size,(double),1)(offset,(double),1)
        (transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvFrame = std::shared_ptr<OpenMBVFrame>(new OpenMBVFrame(size, offset, fmatvec::Vec3({-1,1,1}),
                                                                   transparency, pointSize, lineWidth));
      }
  };

}  

#endif
