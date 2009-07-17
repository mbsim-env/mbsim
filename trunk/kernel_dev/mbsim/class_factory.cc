/* Copyright (C) 2004-2009 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: mfoerg@users.berlios.de
 */

#include <config.h>
#include <mbsim/class_factory.h>
#include <mbsim/object.h>
#include <mbsim/contour.h>
#include <mbsim/contours/point.h>
#include "mbsim/contours/line.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/circle_hollow.h"
#include "mbsim/contours/frustum2d.h"
#include "mbsim/contours/edge.h"
#include "mbsim/contours/area.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/planewithfrustum.h"
#include "mbsim/contours/contour_interpolation.h"
#include "mbsim/contours/contour_quad.h"
#include "mbsim/contours/cuboid.h"
#include "mbsim/contours/compound_contour.h"
#include <mbsim/group.h>
#include <mbsim/tree.h>
#include <mbsim/contact.h>
#include <mbsim/rigid_body.h>
#include <mbsim/constitutive_laws.h>

using namespace std;

namespace MBSim {
  DynamicSystem* ClassFactory::getDynamicSystem(const string &type) {
    if(type == "Group")
      return new Group("NoName");
    else if(type == "Tree")
      return new Tree("NoName");
    return 0;
  }

  Object* ClassFactory::getObject(const string &type) {
    if(type == "RigidBody")
      return new RigidBody("NoName");
    return 0;
  }

  Link* ClassFactory::getLink(const string &type) {
    if(type == "Contact")
      return new Contact("NoName");
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

  GeneralizedForceLaw* ClassFactory::getGeneralizedForceLaw(const string &type) {
    if(type == "UnilateralConstraint")
      return new UnilateralConstraint;
    return 0;
  }

  FrictionForceLaw* ClassFactory::getFrictionForceLaw(const string &type) {
    if(type == "PlanarCoulombFriction")
      return new PlanarCoulombFriction;
    return 0;
  }

  GeneralizedImpactLaw* ClassFactory::getGeneralizedImpactLaw(const string &type) {
    if(type == "UnilateralNewtonImpact")
      return new UnilateralNewtonImpact;
    return 0;
  }
  
  FrictionImpactLaw* ClassFactory::getFrictionImpactLaw(const string &type) {
    if(type == "PlanarCoulombImpact")
      return new PlanarCoulombImpact;
    return 0;
  }

}

