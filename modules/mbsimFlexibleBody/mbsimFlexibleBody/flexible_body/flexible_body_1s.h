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
#include "mbsimFlexibleBody/frames/frame_1s.h"
#include <openmbvcppinterface/spineextrusion.h>

namespace MBSimFlexibleBody {

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
      FlexibleBody1s(const std::string &name, bool openStructure);

      virtual void init(InitStage stage, const MBSim::InitConfigSet &config);
      virtual void plot();

      void setLength(double L_) { L = L_; }
      double getLength() const { return L; }

      bool getOpenStructure() const { return openStructure; }

      void addFrame(Frame1s *frame);
      using FlexibleBodyContinuum<double>::addFrame;

      virtual void updatePositions(Frame1s* frame);
      virtual void updateVelocities(Frame1s* frame);
      virtual void updateAccelerations(Frame1s* frame);
      virtual void updateJacobians(Frame1s* frame, int j=0);
      virtual void updateGyroscopicAccelerations(Frame1s* frame);

      virtual fmatvec::Vec3 getPosition(double s);
      virtual fmatvec::SqrMat3 getOrientation(double s);
      virtual fmatvec::Vec3 getAngles(double s) { return fmatvec::Vec3(); }

      void setOpenMBVSpineExtrusion(const std::shared_ptr<OpenMBV::SpineExtrusion> &body) { openMBVBody=body; }

    protected:
      /**
       * \brief length of beam
       */
      double L;

      /**
       * \brief flag for open (cantilever beam) or closed (rings) structures
       */
      bool openStructure;

      Frame1s P;
  };

}

#endif
