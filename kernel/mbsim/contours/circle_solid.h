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

#ifndef _CIRCLE_SOLID_H_
#define _CIRCLE_SOLID_H_

#include "mbsim/contours/circle.h"

namespace MBSim {

  /**
   * \brief circular contour with contact possibility from outside
   * \author Martin Foerg
   * \date 2009-04-20 some commments (Thorsten Schindler)
   * \date 2009-07-16 new file (Bastian Esefeld)
   * \date 2009-12-21 special circle (Thorsten Schindler)
   */ 
  class CircleSolid : public MBSim::Circle {	
    public:
      /**
       * \brief constructor
       * \param name of circle
       */
      CircleSolid(const std::string& name) : Circle(name,true) {}

      /**
       * \brief constructor
       * \param name of circle
       * \param radius of circle
       */
      CircleSolid(const std::string &name, double r_) : Circle(name,r_,true) {}

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "CircleSolid"; }
      /***************************************************/
  };      
}

#endif /* _CIRCLE_SOLID_H_ */

