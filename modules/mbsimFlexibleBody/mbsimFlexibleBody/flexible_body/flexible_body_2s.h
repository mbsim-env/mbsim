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

#ifndef _FLEXIBLE_BODY_2S_H_
#define _FLEXIBLE_BODY_2S_H_

#include "mbsimFlexibleBody/flexible_body.h"
#include <openmbvcppinterface/spineextrusion.h>

namespace MBSim {
  class ContourFrame;
}

namespace MBSimFlexibleBody {

  class Frame2s;

  /*!
   * \brief tbd
   * \author Martin Foerg
   */
  class FlexibleBody2s : public FlexibleBodyContinuum<fmatvec::Vec2> {
    public:
      /*!
       * \brief constructor:
       * \param name of body
       */
      FlexibleBody2s(const std::string &name) : FlexibleBodyContinuum<fmatvec::Vec2>(name) { } //, L(0), openStructure(openStructure_) { }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void plot() override;

//      void setLength(double L_) { L = L_; }
//      double getLength(){ return L; }

      void addFrame(Frame2s *frame);
      using FlexibleBodyContinuum<fmatvec::Vec2>::addFrame;

      virtual void updatePositions(Frame2s* frame);
      virtual void updateVelocities(Frame2s* frame);
      virtual void updateAccelerations(Frame2s* frame);
      virtual void updateJacobians(Frame2s* frame, int j=0);
      virtual void updateGyroscopicAccelerations(Frame2s* frame);

      void setOpenMBVSpineExtrusion(const std::shared_ptr<OpenMBV::SpineExtrusion> &body) { openMBVBody=body; }

    protected:
//      /**
//       * \brief length of beam
//       */
//      double L;
//
//      /**
//       * \brief flag for open (cantilever beam) or closed (rings) structures
//       */
//      bool openStructure;
  };

}

#endif
