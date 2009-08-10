/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: schneidm@users.berlios.de
 */

#include "valve43.h"
#include "mbsim/data_interface_base.h"
#include "mbsim/userfunction.h"
#include "hydnode.h"
#include "hydline.h"
#include "pressure_loss.h"

using namespace std;

namespace MBSim {

  Valve43::Valve43(valve43Datastruct v, bool setvalued) : Group(v.Name) {

    for (int i=0; i<4; i++) {
      string name;
      UserFunction * fun;
      switch (i) {
        case 0:
          name="lAT";
          fun=v.relAreaAT;
          break;
        case 1:
          name="lPA";
          fun=v.relAreaPA;
          break;
        case 2:
          name="lPB";
          fun=v.relAreaPB;
          break;
        case 3:
          name="lBT";
          fun=v.relAreaBT;
          break;
      };
      HydLineValve * l;
      if (setvalued) 
        l = new HydLineValveBilateral(name);
      else
        l = new HydLineValve(name);
      addObject(l);
      l->setDiameter(v.dValve);
      l->setLength(v.lValve);
      l->addPressureLoss(new PressureLossVarAreaZeta(
            "ValvePressureLoss",
            1./v.alphaValve/v.alphaValve-1.,
            new FuncFunction(fun, v.signal),
            v.minRelArea));
    }

    for (int i=0; i<4; i++) {
      string name;
      switch (i) {
        case 0:
          name=v.NameLineP;
          break;
        case 1:
          name=v.NameLineA;
          break;
        case 2:
          name=v.NameLineB;
          break;
        case 3:
          name=v.NameLineT;
          break;
      };
      HydLine * l = new HydLine(name);
      addObject(l);
      l->setDiameter(v.dLine);
      l->setLength(v.lLine);
    }

    HydNodeRigid * nP = new HydNodeRigid("nP");
    addLink(nP);
    nP->addInFlow(static_cast<HydLine*>(getObject(v.NameLineP)));
    nP->addOutFlow(static_cast<HydLine*>(getObject("lPA")));
    nP->addOutFlow(static_cast<HydLine*>(getObject("lPB")));

    HydNodeRigid * nA = new HydNodeRigid("nA");
    addLink(nA);
    nA->addInFlow(static_cast<HydLine*>(getObject("lPA")));
    nA->addOutFlow(static_cast<HydLine*>(getObject("lAT")));
    nA->addOutFlow(static_cast<HydLine*>(getObject(v.NameLineA)));

    HydNodeRigid * nB = new HydNodeRigid("nB");
    addLink(nB);
    nB->addInFlow(static_cast<HydLine*>(getObject("lPB")));
    nB->addOutFlow(static_cast<HydLine*>(getObject("lBT")));
    nB->addOutFlow(static_cast<HydLine*>(getObject(v.NameLineB)));

    HydNodeRigid * nT = new HydNodeRigid("nT");
    addLink(nT);
    nT->addInFlow(static_cast<HydLine*>(getObject("lAT")));
    nT->addInFlow(static_cast<HydLine*>(getObject("lBT")));
    nT->addOutFlow(static_cast<HydLine*>(getObject(v.NameLineT)));

  };

}
