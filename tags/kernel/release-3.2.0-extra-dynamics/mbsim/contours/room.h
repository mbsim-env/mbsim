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

#include "mbsim/contour.h"
#include "mbsim/contours/compound_contour.h"

namespace MBSim {

  /**
   * \brief Room with 6 faces pointing inwards
   */
  class Room : public CompoundContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Room(const std::string &name);

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const {
        return "Room";
      }
      /***************************************************/

      /* GETTER / SETTER */
      void setLength(double l_) {
        l = l_;
      }
      void setHeight(double h_) {
        h = h_;
      }
      void setDepth(double d_) {
        d = d_;
      }
      /***************************************************/

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBV(bool enable, int number);
#endif

    protected:
      /**
       * \brief length, height and depth of room
       */
      double l, h, d;

#ifdef HAVE_OPENMBVCPPINTERFACE
      /*!
       * \brief enable openMBV output
       */
      bool enable;

      /*!
       * \brief grid size
       */
      int gridSize;

#endif

      void init(InitStage stage);
  };
}

#endif /* _ROOM_H_ */

