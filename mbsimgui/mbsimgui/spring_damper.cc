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
#include "spring_damper.h"
#include "basic_properties.h"
#include "function_properties.h"
#include "kinetics_properties.h"
#include "ombv_properties.h"
#include "function_property_factory.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  SpringDamper::SpringDamper(const string &str) : FrameLink(str) {

//    forceFunction.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory(this),MBSIM%"forceFunction"));
//
//    unloadedLength.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"unloadedLength",vector<string>(2,"m")),"",4));
//
//    coilSpring.setProperty(new CoilSpringMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVCoilSpring",getID()));
  }

  DirectionalSpringDamper::DirectionalSpringDamper(const string &str) : FloatingFrameLink(str) {

//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new VecProperty(3),"-",MBSIM%"forceDirection"));
//    forceDirection.setProperty(new ExtPhysicalVarProperty(input));
//
//    forceFunction.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory(this),MBSIM%"forceFunction"));
//
//    unloadedLength.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"unloadedLength",vector<string>(2,"m")),"",4));
//
//    coilSpring.setProperty(new CoilSpringMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVCoilSpring",getID()));
  }

  GeneralizedSpringDamper::GeneralizedSpringDamper(const string &str) : DualRigidBodyLink(str) {

//    function.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory(this),MBSIM%"generalizedForceFunction"));
//
//    unloadedLength.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("0",MBSIM%"generalizedUnloadedLength",vector<string>(2)),"",4));
  }

}
