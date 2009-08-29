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

#ifndef  _HYDLINE_H_
#define  _HYDLINE_H_

#include "object_hydraulics.h"

namespace MBSim {

  class Frame;
  class HydNode;
  class HydlinePressureloss;
  class PressureLoss;

  class HydLineAbstract : public ObjectHydraulics {
    public:
      HydLineAbstract(const std::string &name);
      ~HydLineAbstract() {};
      virtual std::string getType() const { return "HydLineAbstract"; }

      void setFromNode(HydNode * nFrom_) {nFrom=nFrom_; }
      void setToNode(HydNode * nTo_) {nTo=nTo_; }
      void setLength(double l_) {l=l_; }
      void setDiameter(double d_) {d=d_; }
      void setDirection(fmatvec::Vec dir) {direction=dir/nrm2(dir); }
      HydNode * getFromNode() { return nFrom; }
      HydNode * getToNode() {return nTo; }
      double getDiameter() {return d; }
      double getLength() {return l; }

      virtual fmatvec::Vec getQIn(double t) = 0;
      virtual fmatvec::Vec getQOut(double t) = 0;
      virtual fmatvec::Vec getInflowFactor() = 0;
      virtual fmatvec::Vec getOutflowFactor() = 0;

      void init(InitStage stage);

      void initializeUsingXML(TiXmlElement *element);

    protected:
      HydNode * nFrom;
      HydNode * nTo;
      double d, l;
      double Area, rho;
      fmatvec::Vec direction;
  };

  class HydLine : public HydLineAbstract {
    public:
      HydLine(const std::string &name) : HydLineAbstract(name), MFac(0), pressureLossGravity(0), pressureLoss(NULL) {}
      ~HydLine() {};
      virtual std::string getType() const { return "HydLine"; }

      void addPressureLoss(PressureLoss * dp);

      virtual fmatvec::Vec getQIn(double t) {return u; }
      virtual fmatvec::Vec getQOut(double t) {return -u; }
      virtual fmatvec::Vec getInflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, -1.); }
      virtual fmatvec::Vec getOutflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, 1.); }

      void init(InitStage stage);
      void calcqSize() {qSize=0; }
      void calcuSize(int j) {uSize[j]=1; }

      void updateh(double t);
      void updateM(double t) {M(0,0)=MFac; }

      void plot(double t, double dt);

      void initializeUsingXML(TiXmlElement *element);
    private:
      double MFac;
      double pressureLossGravity;
      HydlinePressureloss * pressureLoss;
  };
}

#endif   /* ----- #ifndef _HYDLINE_H_  ----- */

