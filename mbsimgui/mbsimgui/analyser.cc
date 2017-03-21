/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <config.h>
#include "analyser.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Eigenanalyser::Eigenanalyser() {

//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"s",MBSIMANALYSER%"startTime"));
//    startTime.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"s",MBSIMANALYSER%"endTime"));
//    endTime.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-2"),"s",MBSIMANALYSER%"plotStepSize"));
//    plotStepSize.setProperty(new ExtPhysicalVarProperty(input));
//
//    initialState.setProperty(new ChoiceProperty2(new VecPropertyFactory(0,MBSIMANALYSER%"initialState",vector<string>(3,"")),"",4));
//
//    task.setProperty(new TextProperty("\"eigenmode\"", MBSIMANALYSER%"task"));
//
//    amplitude.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIMANALYSER%"amplitude",vector<string>(2,"")),"",4));
//
//    mode.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIMANALYSER%"mode",vector<string>(2,"")),"",4));
//
//    determineEquilibriumState.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("0",MBSIMANALYSER%"determineEquilibriumState",vector<string>(2,"")),"",4));
  }

  Eigenanalyser::~Eigenanalyser() {
  }

}
