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

#ifndef _FLOATING_CONTOUR_FRAME_H__
#define _FLOATING_CONTOUR_FRAME_H__

#include "mbsim/frames/contour_frame.h"

namespace MBSimFlexibleBody {

  //class Contour1sFlexible;
  class Frame1s;

  /**
   * \brief tbd
   * \author Martin Foerg
   */
  class FloatingContourFrame : public MBSim::ContourFrame {

    public:
      FloatingContourFrame(const std::string &name = "dummy", Frame1s* R_ = NULL) : ContourFrame(name), R(R_) { }

      std::string getType() const { return "FloatingContourFrame"; }

      void setRelativePosition(const fmatvec::Vec3 &r) { RrRP = r; }
      void setRelativeOrientation(const fmatvec::SqrMat3 &A) { ARP = A; }

      void setFrameOfReference(Frame1s *R_) { R = R_; }

      const fmatvec::Vec3& getGlobalRelativePosition(double t);

      void updatePositions(double t);
      void updateVelocities(double t);
      void updateAccelerations(double t);
      void updateJacobians(double t, int j=0);
      void updateGyroscopicAccelerations(double t);

    protected:
      Frame1s *R;
      fmatvec::Vec3 RrRP, WrRP;
      fmatvec::SqrMat3 ARP;
  };

}

#endif

