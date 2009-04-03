/* Copyright (C) 2006 Mathias Bachmayer
 
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
 * Institute of Applied Mechanics
 * Technical University of Munich
 *
 * Contact:
 *    rzander@users.berlios.de
 *
 */ 
#ifndef _EDInterface_H_
#define _EDInterface_H_
#include <mbsim/element.h>
#include "hdf5serie/vectorserie.h"

namespace MBSim {

  class DynamicSystem;

  /*! base class for all dynamic systems with own dynamics not being solid-mechanics or hydraulics */
  class ExtraDynamicInterface : public Element {

    protected:
      DynamicSystem* parent;

      fmatvec::Vec x, xd, x0;
      int xSize;
      int xInd;
      fmatvec::Vec y;

    public:

      ExtraDynamicInterface(const std::string &name);
      ExtraDynamicInterface(const std::string &name, int xSize_);

      const fmatvec::Vec& getx() const {return x;}
      fmatvec::Vec& getx() {return x;}

      virtual void writex();
      virtual void readx0(); 
      virtual void updatexRef(const fmatvec::Vec& ref);
      virtual void updatexdRef(const fmatvec::Vec& ref);
      void setxInd(int xInd_) {xInd = xInd_;};
      int getxSize() const {return xSize;}
      virtual void init();
      virtual void preinit() {}
      virtual void initz();
      virtual void calcxSize() {};
      virtual void updateg(double t) {};
      virtual void updategd(double t) {};
      virtual void updateKinetics(double t) {};
      virtual void updatedx(double t, double dt) {};
      virtual void updatexd(double t) {};

      virtual void plot(double t, double dt = 1); 
      virtual void initPlot();
      virtual void closePlot(); 

      void setParent(DynamicSystem *parent_) {parent = parent_;}

      void setx0(fmatvec::Vec x_){x0=x_;}
  };

}

#endif
