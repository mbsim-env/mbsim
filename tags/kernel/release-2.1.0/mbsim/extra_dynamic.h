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
 * Contact: mbachmayer@gmx.de
 */

#ifndef _EXTRA_DYNAMIC_H_
#define _EXTRA_DYNAMIC_H_

#include "mbsim/element.h"
#include "mbsim/extradynamic_interface.h"
#include "hdf5serie/vectorserie.h"

namespace MBSim {

  class DynamicSystem;
  class Link;

  /**
   * \brief base class for dynamic systems of the form \f$\dot{x}=f\left(x,u\right)\f$ and \f$y=g\left(x,u\right)\f$
   * \author Mathias Bachmayer
   * \date 2009-04-06 interface extracted (Thorsten Schindler)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   */
  class ExtraDynamic : public Element, ExtraDynamicInterface {
    public:
      /**
       * \brief constructor
       * \param name of extra dynamic system
       */
      ExtraDynamic(const std::string &name);

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      using ExtraDynamicInterface::updatedx;
      using ExtraDynamicInterface::updatexd;
      virtual void calcxSize() {};
      virtual const fmatvec::Vec& getx() const { return x; }
      virtual fmatvec::Vec& getx() { return x; }
      virtual void setxInd(int xInd_) { xInd = xInd_; };
      virtual int getxSize() const { return xSize; }
      virtual void updatexRef(const fmatvec::Vec& ref);
      virtual void updatexdRef(const fmatvec::Vec& ref);
      virtual void init(InitStage stage);
      virtual void initz();
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "ExtraDynamic"; }
      virtual void closePlot(); 
      virtual void plot(double t, double dt = 1); 
      /***************************************************/

      /* INTERFACE TO BE DEFINED IN DERIVED CLASSES */
      virtual void updateg(double t) = 0;
      /***************************************************/

      /* GETTER / SETTER */
      void setParent(DynamicSystem *parent_) { parent = parent_; }
      DynamicSystem* getParent() { return parent; }
      void setx0(fmatvec::Vec x_) { x0 = x_; }
      virtual Element *getByPathSearch(std::string path);
      /***************************************************/

    protected:
      /**
       * \brief dynamic system parent for plotting
       */
      DynamicSystem* parent;

      /**
       * \brief order one parameter, differentiated order one parameter, initial order one parameters
       */
      fmatvec::Vec x, xd, x0;

      /**
       * \brief size of order one parameter vector
       */
      int xSize;

      /**
       * \brief index of order one parameters
       */
      int xInd;

      /**
       * \brief system output \f$y=g\left(x\right)\f$
       */
      fmatvec::Vec y;

  };

}

#endif /* _EXTRA_DYNAMIC_H_ */

