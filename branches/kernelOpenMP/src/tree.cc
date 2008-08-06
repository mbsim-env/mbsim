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
#include "tree.h"
#include "body.h"

namespace MBSim {

  Tree::Tree(const string &projectName) : Object(projectName), lSize(0) {
  }

  Tree::~Tree() { 
  port.clear();			//sonst: object Destructor loescht Ports, zweiter delete Aufruf (z.B. ~BodyRigidRel) liefert segmentation fault
  contour.clear();
  }

  void Tree::addPort(Port * port_)  {
    port.push_back(port_);
  }

  void Tree::addContour(Contour* contour_) {
    contour.push_back(contour_);
  }

  void Tree::plotParameterFiles() {
	Object::plotParameterFiles();
	getRoot()->plotParameterFiles();
  }

}
