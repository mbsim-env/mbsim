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

#ifndef _SOLID_CIRCLE_H_
#define _SOLID_CIRCLE_H_

#include "mbsim/contours/circle.h"

namespace MBSim {

  /**
   * \brief circular contour with contact possibility from outside
   * \author Martin Foerg
   * \date 2009-04-20 some commments (Thorsten Schindler)
   * \date 2009-07-16 new file (Bastian Esefeld)
   * \date 2009-12-21 special circle (Thorsten Schindler)
   */ 
  class SolidCircle : public MBSim::Circle {	
    public:
      /**
       * \brief constructor
       * \param name of circle
       */
      SolidCircle(const std::string& name="", Frame *R=0) : Circle(name,true,R) {}

      /**
       * \brief constructor
       * \param name of circle
       * \param radius of circle
       */
      SolidCircle(const std::string &name, double r_, Frame *R=0) : Circle(name,r_,true,R) {}

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "SolidCircle"; }
      /***************************************************/
  };      
}

#endif /* _CIRCLE_SOLID_H_ */

