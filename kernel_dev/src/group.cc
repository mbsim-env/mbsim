/* Copyright (C) 2004-2008  Martin Förg
 
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
#include "group.h"
#include "extra_dynamic_interface.h"

namespace MBSim {

  Group::Group(const string &name) : Subsystem(name) {
  }

  Group::~Group() {
  }

  void Group::facLLM() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i)
      (*i)->facLLM();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).facLLM();
  }

  void Group::updatezd(double t) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatezd(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatezd(t);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexd(t);

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatexd(t);

  }

  void Group::updatedu(double t, double dt) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatedu(t,dt);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updatedu(t,dt);

  }

}
