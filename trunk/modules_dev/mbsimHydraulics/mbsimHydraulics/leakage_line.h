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

#ifndef  _LEAKAGE_LINE_H_
#define  _LEAKAGE_LINE_H_

#include "mbsimHydraulics/hline.h"

namespace MBSim {

  class PlaneLeakagePressureLoss;
  class CircularLeakagePressureLoss;

  /*! PlaneLeakage */
  class PlaneLeakage : public RigidHLine {
    public:
      PlaneLeakage(const std::string &name) : RigidHLine(name), hGap(0), wGap(0) {};
      virtual std::string getType() const { return "PlaneLeakage"; }

      void init(InitStage stage);

      void setGapWidth(double wGap_) {wGap=wGap_; }
      double getGapWidth() {return wGap; }
      void setGapHeight(double hGap_) {hGap=hGap_; }
      double getGapHeight() {return hGap; }
      void addPressureLoss(PlaneLeakagePressureLoss * dp);

      void initializeUsingXML(TiXmlElement * element);

    private:
      double hGap, wGap;
  };

  /*! CircularLeakage */
  class CircularLeakage : public RigidHLine {
    public:
      CircularLeakage(const std::string &name) : RigidHLine(name), rI(0), rO(0), hGap(0) {}
      virtual std::string getType() const { return "CircularLeakage"; }

      void init(InitStage stage);

      void setInnerRadius(double rI_) {rI=rI_; }
      double getInnerRadius() {return rI; }
      void setGapHeight(double hGap_) {hGap=hGap_; }
      double getGapHeight() {return hGap; }
      double getOuterRadius() {return rO; }
      void addPressureLoss(CircularLeakagePressureLoss * dp);

      void initializeUsingXML(TiXmlElement * element);

    private:
      double rI, rO, hGap;

  };

}

#endif   /* ----- #ifndef _LEAKAGE_H_  ----- */

