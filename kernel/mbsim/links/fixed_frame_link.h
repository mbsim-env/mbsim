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

namespace OpenMBV {
  class Group;
  class Arrow;
}

namespace H5 {
  class Group;
}

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

      virtual void init(InitStage stage);

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "FixedFrameLink"; }
      /***************************************************/

      void updateh(int i=0);
      void updatedhdz();

      /* INHERITED INTERFACE OF LINK */
      virtual void updateWRef(const fmatvec::Mat& ref, int i=0);
      virtual void updateVRef(const fmatvec::Mat& ref, int i=0);
      virtual void updatehRef(const fmatvec::Vec &hRef, int i=0);
      virtual void updatedhdqRef(const fmatvec::Mat& ref, int i=0);
      virtual void updatedhduRef(const fmatvec::SqrMat& ref, int i=0);
      virtual void updatedhdtRef(const fmatvec::Vec& ref, int i=0);
      virtual void updaterRef(const fmatvec::Vec &ref, int i=0);
      /***************************************************/

      void updatePositions();
      void updateVelocities();
      void updateGeneralizedPositions();
      void updateGeneralizedVelocities();
      void updateGeneralizedForces();
      void updateForceDirections();
      void updateR();
  };
}

#endif
