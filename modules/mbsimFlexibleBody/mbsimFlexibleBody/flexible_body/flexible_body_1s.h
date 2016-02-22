/* Copyright (C) 2004-2015 MBSim Development Team
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

#ifndef _FLEXIBLE_BODY_1S_H_
#define _FLEXIBLE_BODY_1S_H_

#include "mbsimFlexibleBody/flexible_body.h"

namespace MBSimFlexibleBody {

  class Frame1s;

  /*!
   * \brief tbd
   * \author Martin Foerg
   */
  class FlexibleBody1s : public FlexibleBodyContinuum<double> {
    public:
      /*!
       * \brief constructor:
       * \param name of body
       */
      FlexibleBody1s(const std::string &name) : FlexibleBodyContinuum<double>(name) { }

      void addFrame(Frame1s *frame);

      virtual fmatvec::Vec3 getPosition(double t, double s);
      virtual fmatvec::SqrMat3 getOrientation(double t, double s);
      virtual fmatvec::Vec3 getWs(double t, double s);

      virtual void updatePositions(double t, Frame1s* frame);
      virtual void updateVelocities(double t, Frame1s* frame);
      virtual void updateAccelerations(double t, Frame1s* frame);
      virtual void updateJacobians(double t, Frame1s* frame, int j=0);
      virtual void updateGyroscopicAccelerations(double t, Frame1s* frame);
  };

}

#endif
