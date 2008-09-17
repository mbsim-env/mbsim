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
#include "class_factory.h"
#include "object.h"
#include "rigid_contact.h"
#include "flexible_contact.h"
#include "contour.h"
#include "group.h"
#include "rigid_body.h"

namespace MBSim {
  Object* ClassFactory::getObject(const string &type) {
    if(type == "Group")
      return new Group("NoName");
    else if(type == "RigidBody")
      return new RigidBody("NoName");
    return 0;
  }
  Link* ClassFactory::getLink(const string &type) {
    if(type == "RigidContact")
      return new RigidContact("NoName");
    else if(type == "FlexibleContact")
      return new FlexibleContact("NoName");
    return 0;
  }
  Contour* ClassFactory::getContour(const string &type) {
    if(type == "Point")
      return new Point("NoName");
    else if(type == "Line")
      return new Line("NoName");
    return 0;
  }
  Translation* ClassFactory::getTranslation(const string &type) {
    if(type == "LinearTranslation")
      return new LinearTranslation;
    return 0;
  }
  Rotation* ClassFactory::getRotation(const string &type) {
    if(type == "RotationAboutFixedAxis")
      return new RotationAboutFixedAxis;
    else if(type == "CardanAngles")
      return new CardanAngles;
    return 0;
  }

}





