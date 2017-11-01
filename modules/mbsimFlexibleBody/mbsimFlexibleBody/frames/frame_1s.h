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

#ifndef _FRAME_1S_H__
#define _FRAME_1S_H__

#include "mbsim/frames/frame.h"

namespace MBSimFlexibleBody {

  class Frame1s : public MBSim::Frame {

    public:
      Frame1s(const std::string &name = "dummy", double s_ = 0) : Frame(name), s(s_) { }

      void setParameter(double s_) { s = s_; }
      double getParameter() const { return s; }

      void updatePositions();
      void updateVelocities();
      void updateAccelerations();
      void updateJacobians(int j=0);
      void updateGyroscopicAccelerations();

      virtual void initializeUsingXML(xercesc::DOMElement *element);

    protected:
      double s;
  };

}

#endif
