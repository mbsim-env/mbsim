/* Copyright (C) 2004-2006  Martin Förg
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */
#include <config.h>
#include "connection_flexible.h"
#include "port.h"

namespace MBSim {

  ConnectionFlexible::ConnectionFlexible(const string &name) : Connection(name,false), cT(0), dT(0), cR(0), dR(0) {
    for(int i=0; i<2 ; i++) {
      load.push_back(Vec(6));
      WF[i] >> load[i](Index(0,2));
      WM[i] >> load[i](Index(3,5));
    }
  }

  void ConnectionFlexible::updateKinetics(double t) {
    la(IT) = -cT*g(IT) - dT*gd(IT);
    la(IR) = -cR*g(IR) - dR*gd(IR);
    WF[1] = Wf*la(IT);
    WM[1] = Wm*la(IR);
    WF[0] = -WF[1];
    WM[0] = -WM[1];
    if(KOSYID)
      WM[0] += crossProduct(WrP0P1,WF[0]);
  }

  double ConnectionFlexible::computePotentialEnergy() {
//    double V = 0.5* (trans(la)*g); // ERROR!!! mixed term including velocity*dampingCoefficient
//    return C;
    return 0.5* ( cT*trans(g(IT))*g(IT) + cR*trans(g(IR))*g(IR) );
  }

}
