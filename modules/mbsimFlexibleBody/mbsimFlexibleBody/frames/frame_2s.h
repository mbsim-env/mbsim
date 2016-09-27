/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _FRAME_2S_H__
#define _FRAME_2S_H__

#include "mbsim/frames/frame.h"

namespace MBSimFlexibleBody {

  class Frame2s : public MBSim::Frame {

    public:
      Frame2s(const std::string &name = "dummy", const fmatvec::Vec2 &s_ = fmatvec::Vec2()) : Frame(name), s(s_) { }

      std::string getType() const { return "Frame2s"; }

      void setParameters(const fmatvec::Vec2 &s_) { s = s_; }
      const fmatvec::Vec2& getParameters() const { return s; }

      void updatePositions();
      void updateVelocities();
      void updateAccelerations();
      void updateJacobians(int j=0);
      void updateGyroscopicAccelerations();

      virtual void initializeUsingXML(xercesc::DOMElement *element);

    protected:
      fmatvec::Vec2 s;
  };

}

#endif
