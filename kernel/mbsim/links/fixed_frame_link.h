/* Copyright (C) 2004-2016 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _FIXED_FRAME_LINK_H_
#define _FIXED_FRAME_LINK_H_

#include "mbsim/links/frame_link.h"

namespace MBSim {
  /** 
   * \brief frame link
   * \author Martin Foerg
   */
  class FixedFrameLink : public FrameLink {
    public:
      /**
       * \brief constructor
       * \param name of link machanics
       */
      FixedFrameLink(const std::string &name);

      void calcSize();

      virtual void init(InitStage stage);

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "FixedFrameLink"; }
      /***************************************************/

      void updateh(int i=0);

      const fmatvec::Mat3xV& evalGlobalForceDirection(int i=0) { if(updDF) updateForceDirections(); return DF; }
      const fmatvec::Mat3xV& evalGlobalMomentDirection(int i=0) { if(updDF) updateForceDirections(); return DM; }

      void updateSize();

      void resetUpToDate();
      void updatePositions();
      void updateVelocities();
      void updateGeneralizedPositions();
      void updateGeneralizedVelocities();
      void updateForceDirections();
      void updateForce();
      void updateMoment();
      void updateR();

    protected:
      fmatvec::Mat3xV DF, DM;

      bool updDF;
  };
}

#endif
