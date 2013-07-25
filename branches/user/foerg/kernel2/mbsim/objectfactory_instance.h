/* Copyright (C) 2004-2010 MBSim Development Team
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
 * Contact: friedrich.at.gc@googlemail.com
 */

/* Include this file if you must provide a explicit instantation of the
 * ObjectFactory for a new BaseType */

// NOTE: We can skip this code completely if everything is derived from e.g. 
// class Atom.

#include <mbsim/objectfactory.h>

namespace MBSim {

template<class BaseType>
ObjectFactory<BaseType>& ObjectFactory<BaseType>::instance() {
  static ObjectFactory<BaseType> of;
  return of;
}

}
