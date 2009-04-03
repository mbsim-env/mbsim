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

#ifndef _CLASS_FACTORY_H_
#define _CLASS_FACTORY_H_

#include <string>

namespace MBSim {
  class DynamicSystem;
  class Object;
  class Link;
  class Contour;
  class Translation;
  class Rotation;
  class GeneralizedForceLaw;
  class FrictionForceLaw;
  class GeneralizedImpactLaw;
  class FrictionImpactLaw;

  /**
   * \brief creates default MBSim incredients for topological load mechanism
   * \author Martin Foerg
   * \date 2009-03-25 some comments (Thorsten Schindler)
   */
  class ClassFactory {
    public:
      /* GETTER */
      DynamicSystem* getDynamicSystem(const std::string &type);
      Object* getObject(const std::string &type);
      Link* getLink(const std::string &type);
      Contour* getContour(const std::string &type);
      Translation* getTranslation(const std::string &type);
      Rotation* getRotation(const std::string &type);
      GeneralizedForceLaw* getGeneralizedForceLaw(const std::string &type);
      FrictionForceLaw* getFrictionForceLaw(const std::string &type);
      GeneralizedImpactLaw* getGeneralizedImpactLaw(const std::string &type);
      FrictionImpactLaw* getFrictionImpactLaw(const std::string &type);
  };

}

#endif

