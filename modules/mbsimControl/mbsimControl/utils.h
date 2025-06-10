/* Copyright (C) 2004-2025 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _UTILS_H_
#define _UTILS_H_

#include <string>
#include <vector>
#include <mbsim/element.h>

namespace MBSim {

  class DynamicSystemSolver;

}

namespace MBSimControl {

  class ExternSignalSource;
  class ExternSignalSink;

  ExternSignalSource *getExternSignalSource(const MBSim::DynamicSystemSolver *dss, int i=0);
  ExternSignalSource *getExternSignalSource(const MBSim::DynamicSystemSolver *dss, const std::string &path);
  ExternSignalSink *getExternSignalSink(const MBSim::DynamicSystemSolver *dss, int i=0);
  ExternSignalSink *getExternSignalSink(const MBSim::DynamicSystemSolver *dss, const std::string &path);

}

#endif
