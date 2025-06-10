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


#include <config.h>
#include "mbsimControl/utils.h"
#include "mbsimControl/extern_signal_source.h"
#include "mbsimControl/extern_signal_sink.h"
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace MBSim;

namespace MBSimControl {

  ExternSignalSource *getExternSignalSource(const DynamicSystemSolver *dss, int i) {
    vector<Link*> links = dss->getLinks();
    int j=0;
    for(auto & link : links) {
      if(dynamic_cast<ExternSignalSource*>(link)) {
        if(j==i)
          return static_cast<ExternSignalSource*>(link);
        else
          j++;
      }
    }
    return nullptr;
  }

  ExternSignalSource *getExternSignalSource(const DynamicSystemSolver *dss, const string &path) {
    vector<Link*> links = dss->getLinks();
    for(auto & link : links) {
      if(dynamic_cast<ExternSignalSource*>(link)) {
        if(link->getPath()==path)
          return static_cast<ExternSignalSource*>(link);
      }
    }
    return nullptr;
  }

  ExternSignalSink *getExternSignalSink(const DynamicSystemSolver *dss, int i) {
    vector<Link*> links = dss->getLinks();
    int j=0;
    for(auto & link : links) {
      if(dynamic_cast<ExternSignalSink*>(link)) {
        if(j==i)
          return static_cast<ExternSignalSink*>(link);
        else
          j++;
      }
    }
    return nullptr;
  }

  ExternSignalSink *getExternSignalSink(const DynamicSystemSolver *dss, const string &path) {
    vector<Link*> links = dss->getLinks();
    for(auto & link : links) {
      if(dynamic_cast<ExternSignalSink*>(link)) {
        if(link->getPath()==path)
          return static_cast<ExternSignalSink*>(link);
      }
    }
    return nullptr;
  }
}

