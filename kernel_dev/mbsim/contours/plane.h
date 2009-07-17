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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _PLANE_H_
#define _PLANE_H_

#include "mbsim/contour.h"

namespace MBSim {


  /** 
   * \brief plane without borders
   * \author Martin Foerg
   * \date 2009-03-23 some comments (Thorsten Schindler)
   *
   * normal equals first column in orientation matrix
   */
  class Plane : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Plane(const std::string &name) : RigidContour(name) {}
  };
}

#endif /* _PLANE_H_ */
