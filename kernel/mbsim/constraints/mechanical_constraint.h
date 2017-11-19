/* Copyright (C) 2004-2017 MBSim Development Team
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

#ifndef _MECHANICAL_CONSTRAINT_H
#define _MECHANICAL_CONSTRAINT_H

#include "constraint.h"

namespace MBSim {

  class MechanicalLink;

  /** 
   * \brief Class for mechanical constraints
   * \author Martin Foerg
   */
  class MechanicalConstraint : public Constraint {
    public:
      MechanicalConstraint(const std::string &name) : Constraint(name), link(nullptr) { }
      MechanicalLink *getMechanicalLink() { return link; }

    protected:
      MechanicalLink *link;
  };

}

#endif
