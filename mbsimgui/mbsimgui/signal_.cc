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
#include "signal_.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Signal::Signal(const QString &str) : Link(str) {
  }

  Signal::~Signal() {
  }

  PIDController::PIDController(const QString &str) : Signal(str) {
//    sRef.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROL%"inputSignal"));
//    sdRef.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROL%"derivativeOfInputSignal"));
//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"-",MBSIMCONTROL%"P"));
//    P.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMCONTROL%"I"));
//    I.setProperty(new ExtPhysicalVarProperty(input));
//
//    input.clear();
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMCONTROL%"D"));
//    D.setProperty(new ExtPhysicalVarProperty(input));
  }

  UnarySignalOperation::UnarySignalOperation(const QString &str) : Signal(str) {
//    sRef.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROL%"inputSignal"));

//    f.setProperty(new ChoiceProperty2(new SymbolicFunctionPropertyFactory3(this,"VV",vector<string>(1,"x")),MBSIMCONTROL%"function"));
  }

  BinarySignalOperation::BinarySignalOperation(const QString &str) : Signal(str) {
//    s1Ref.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROL%"firstInputSignal"));
//    s2Ref.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROL%"secondInputSignal"));
//
//    vector<string> var;
//    var.push_back("x1");
//    var.push_back("x2");
//    f.setProperty(new ChoiceProperty2(new SymbolicFunctionPropertyFactory2(this,"VVV",var),MBSIMCONTROL%"function"));
  }

  ExternSignalSource::ExternSignalSource(const QString &str) : Signal(str) {
//    sourceSize.setProperty(new IntegerProperty(1,MBSIMCONTROL%"sourceSize"));
  }

  ExternSignalSink::ExternSignalSink(const QString &str) : Signal(str) {
//    inputSignal.setProperty(new SignalOfReferenceProperty("",this,MBSIMCONTROL%"inputSignal"));
  }

}
