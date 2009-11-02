/* Copyright (C) 2006  Martin Förg, Roland Zander
 
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
 *   rzander@users.berlios.de
 *
 */
#include<config.h>
#include "hitsphere_link.h"
#include "multi_body_system.h"

namespace MBSim {

  HitSphereLink::HitSphereLink() : active(false) {}

  void HitSphereLink::init() {
    mbs = obj[0]->getMbs();
  }

  void HitSphereLink::setParents(Object *obj0, Object *obj1, Link* link) {
    obj[0] = obj0;
    obj[1] = obj1;

    linkList.push_back(link);
  }

  void HitSphereLink::checkActive() {
    Vec WrOC[2];
    double R[2];

    WrOC[0] = obj[0]->getWrHitSphere();
    R   [0] = obj[0]->getRadiusHitSphere();
    WrOC[1] = obj[1]->getWrHitSphere();
    R   [1] = obj[1]->getRadiusHitSphere();

    if(nrm2(WrOC[0] - WrOC[1]) < R[0] + R[1] ) {
      for(vector<Link*>::iterator iL = linkList.begin(); iL != linkList.end(); ++iL) {
		if((*iL)->isSetValued()) {
                  if ((*iL)->isBilateral()) mbs->linkSetValuedBilateral.push_back(*iL); 
		  else mbs->linkSetValuedUnilateral.push_back(*iL); 
		}
		else {
		  mbs->linkSingleValued.push_back(*iL);
		}
      }
    }
  }

}
