/* Copyright (C) 2004-2012 MBSim Development Team
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

#include <config.h>

#include "multi_dimensional_newton_method.h"

#include <mbsim/utils/eps.h>

#include <iostream>

using namespace fmatvec;
using namespace std;

namespace fmatvec {
  bool operator<(const Index & i1, const Index & i2) {
      if(i1.start() < i2.start())
        return true;
      else if(i1.start() > i2.start())
        return false;
      else {
        if(i1.end() < i2.end())
          return true;
        else
          return false;
      }
  }
}

/*MultiDimensionalNewtonMethod*/
namespace MBSim {

}
