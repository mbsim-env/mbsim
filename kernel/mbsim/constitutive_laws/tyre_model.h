/* Copyright (C) 2004-2022 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _TYRE_MODEL_H_
#define _TYRE_MODEL_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  class TyreModel : public Element {
    public:
      TyreModel() : Element(uniqueDummyName(this)) {
        plotFeature[plotRecursive]=false;
      }
      ~TyreModel() override = default;

      virtual void initPlot(std::vector<std::string> &plotColumns) { }
      virtual void plot(std::vector<double> &plotVector) { }
      using Element::plot;

      virtual int getxSize() const { return 0; }
      virtual int getDMSize() const { return 1; }
      virtual void updatexd() { }

      virtual void updateGeneralizedForces() { }

      virtual int getDataSize() const = 0;
      virtual fmatvec::VecV getData() const = 0;

      virtual bool motorcycleKinematics() const = 0;

      virtual fmatvec::Vec2 getEllipseParameters() const = 0;
      virtual double evalFreeRadius() = 0;
  };

}

#endif
