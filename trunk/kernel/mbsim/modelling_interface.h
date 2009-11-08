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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _MODELLING_INTERFACE_H_
#define _MODELLING_INTERFACE_H_

namespace MBSim {

  class Object;
  class Link;
  class DynamicSystem;

  /*!
   * \brief Interface for models of arbitrary domains, e.g. electrical components 
   * \author Martin Foerg
   * \date 2009-05-04 initial commit (Martin Foerg)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   */
  class ModellingInterface {
    public:

      /*!
       * \brief Destructor.
       */
      virtual ~ModellingInterface() {};

      /**
       * \brief initialize object at start of simulation with respect to contours and frames
       */
      virtual void init(InitStage stage) = 0;

      /*!
       * \brief Get the name of the model 
       * \return The name 
       */
      virtual std::string getName() const = 0;

      /*!
       * \brief Set the name of the model 
       * \param name The name 
       */
      virtual void setName(std::string name) = 0;

      virtual DynamicSystem* getParent() = 0;
      virtual void setParent(DynamicSystem* sys) = 0; 

      /*!
       * \brief Process all models of the same type as the calling model. 
       * \param modellList On input, modellList contains all models. On output,
       * modellList contains all models that are not processed, i.e. all
       * models of different type.
       * \param objectList On output, objectList contains all models that are objects.
       * \param linkList On output, linkList contains all models that are links.
       */
      virtual void processModellList(std::vector<ModellingInterface*> &modellList, std::vector<MBSim::Object*> &objectList, std::vector<MBSim::Link*> &linkList) = 0;
  };

}

#endif /* _MODELLING_INTERFACE_H_ */

