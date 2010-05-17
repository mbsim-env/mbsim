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

#ifndef _CIRCLE_HOLLOW_H_
#define _CIRCLE_HOLLOW_H_

#include "mbsim/contours/circle.h"

namespace MBSim {

  /**
   * \brief circle describing contact from inside
   * \author Roland Zander
   * \date 2009-04-20 some comments (Thorsten Schindler)
   * \date 2009-05-13 OpenMBV-Interface (Bastian Esefeld)
   * \date 2009-07-16 new file (Bastian Esefeld)
   * \date 2009-12-21 special circle (Thorsten Schindler)
   */
  class CircleHollow : public Circle {
    public:
      /**
       * \brief constructor
       * \param name of circle
       */
      CircleHollow(const std::string& name) : Circle(name) {}

      /**
       * \brief constructor
       * \param name of circle
       * \param radius of circle
       */
      CircleHollow(const std::string &name, double r_) : Circle(name,r,false) {}

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "CircleHollow"; }
      /***************************************************/
  };      
}

#endif /* _CIRCLE_HOLLOW_H_ */

