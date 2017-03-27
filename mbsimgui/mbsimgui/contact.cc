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
#include "contact.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Contact::Contact(const QString &str) : Link(str) {

//    connections.setProperty(new ConnectContoursProperty(2,this));
//
//    contactForceLaw.setProperty(new GeneralizedForceLawChoiceProperty(this,MBSIM%"normalForceLaw"));
//
//    contactImpactLaw.setProperty(new GeneralizedImpactLawChoiceProperty(this,MBSIM%"normalImpactLaw"));
//
//    frictionForceLaw.setProperty(new FrictionForceLawChoiceProperty(this,MBSIM%"tangentialForceLaw"));
//
//    frictionImpactLaw.setProperty(new FrictionImpactLawChoiceProperty(this,MBSIM%"tangentialImpactLaw"));
//
//    searchAllContactPoints.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"searchAllContactPoints",vector<string>(2,"")),"",4));
//
//    initialGuess.setProperty(new ChoiceProperty2(new VecPropertyFactory(0,MBSIM%"initialGuess",vector<string>(3,"")),"",4));
  }

}
