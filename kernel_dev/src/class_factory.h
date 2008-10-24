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

#ifndef _CLASS_FACTORY_H_
#define _CLASS_FACTORY_H_

#include <string>

using namespace std;

namespace MBSim {
  class Subsystem;
  class Object;
  class Link;
  class Contour;
  class Translation;
  class Rotation;
  class ConstraintLaw;
  class DryFriction;
  class NormalImpactLaw;
  class TangentialImpactLaw;

  class ClassFactory {
    public:
      Subsystem* getSubsystem(const string &type);
      Object* getObject(const string &type);
      Link* getLink(const string &type);
      Contour* getContour(const string &type);
      Translation* getTranslation(const string &type);
      Rotation* getRotation(const string &type);
      ConstraintLaw* getConstraintLaw(const string &type);
      DryFriction* getFrictionLaw(const string &type);
      NormalImpactLaw* getNormalImpactLaw(const string &type);
      TangentialImpactLaw* getTangentialImpactLaw(const string &type);
  };

}

#endif

