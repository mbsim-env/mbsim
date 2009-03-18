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

#ifndef _BODY_H_
#define _BODY_H_

#include <mbsim/object.h>

namespace MBSim {

  /** 
   *  \brief base class for all mechanical bodies with mass and generalised coordinates
   *  \author Martin Foerg
   *  \date 18.03.09
   */
  class Body : public Object {
    public:
      /**
       * \brief constructor
       * \param name of body
       */
      Body(const string &name);

      /* INHERITED INTERFACE */
      void init();

  };

}

#endif

