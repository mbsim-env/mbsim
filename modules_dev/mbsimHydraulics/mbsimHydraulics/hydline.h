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
 * Contact: schneidm@users.berlios.de
 */

#ifndef  _RIGID_LINE_H_
#define  _RIGID_LINE_H_

#include "mbsimHydraulics/hline.h"

namespace MBSim {

  class Frame;
  class HNode;
  class HydlinePressureloss;
  class LinePressureLoss;

  class RigidLine : public RigidHLine {
    public:
      RigidLine(const std::string &name) : RigidHLine(name), diameter(0) {}
      ~RigidLine() {};
      virtual std::string getType() const { return "RigidLine"; }

      void setDiameter(double diameter_) {diameter=diameter_; }
      double getDiameter() {return diameter; }
      void addPressureLoss(LinePressureLoss * dp);

      void init(InitStage stage);

      void initializeUsingXML(TiXmlElement *element);
    private:
      double diameter;
  };
}

#endif   /* ----- #ifndef _RIGID_LINE_H_  ----- */

