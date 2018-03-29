/* Copyright (C) 2004-2009 MBSim Development Team
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

#ifndef _ROOM_H_
#define _ROOM_H_

#include "mbsim/contours/compound_contour.h"

#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  /**
   * \brief Room with 6 faces pointing inwards
   */
  class Room : public CompoundContour {
    public:
      /**
       * \brief constructor
       * \param name of room
       * \param length of room
       * \param R frame of reference
       */
      Room(const std::string &name="", const fmatvec::Vec3 &length=fmatvec::Vec3(fmatvec::INIT,1), Frame *R=nullptr) : CompoundContour(name,R), l(length(0)), h(length(1)), d(length(2)) { }

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/
      void init(InitStage stage, const InitConfigSet &config) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setLength(const fmatvec::Vec3 &length) { l = length(0); h = length(1); d = length(2); }
      void setLength(double l_, double h_, double d_) { l = l_; h = h_; d = d_; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVCuboid ombv(fmatvec::Vec3(),diffuseColor,transparency);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }

    protected:
      /**
       * \brief length, height and depth of room
       */
      double l{1};
      double h{1};
      double d{1};

      /*!
       * \brief enable openMBV output
       */
      bool enable{false};

      /*!
       * \brief grid size
       */
      int gridSize{10};
  };
}

#endif /* _ROOM_H_ */
