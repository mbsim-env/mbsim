/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#include "fmi_scalar_variable_container.h"

namespace fmi {

template<> template<>
void ScalarVariableContainer<fmiReal>::add<MBSim::Link*>(MBSim::Link* in) {
  if(MBSim::ExternGeneralizedIO* out = dynamic_cast<MBSim::ExternGeneralizedIO*>(in)) {
    addCommon(out);
  }
  if(MBSimControl::ExternSignalSource* out = dynamic_cast<MBSimControl::ExternSignalSource*>(in)) {
    addCommon(out);
  }
  if(MBSimControl::ExternSignalSink* out = dynamic_cast<MBSimControl::ExternSignalSink*>(in)) {
    addCommon(out);
  }
}

}


