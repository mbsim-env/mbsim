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

#ifndef  _HLINE_H_
#define  _HLINE_H_

#include "mbsim/object.h"

namespace MBSimControl {
  class Signal;
}

namespace MBSimHydraulics {

  class HNode;
  class HydlinePressureloss;
  class PressureLoss;

  /*! HLine */
  class HLine : public MBSim::Object {
    public:
      HLine(const std::string &name) : MBSim::Object(name), nFrom(NULL), nTo(NULL), direction(3), Mlocal(0) {};
      virtual std::string getType() const { return "HLine"; }

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateStateDependentVariables(double t) {};
      virtual void updateJacobians(double t) {};
      virtual void updateInverseKineticsJacobians(double t) {};
#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual OpenMBV::Group* getOpenMBVGrp() { return 0; }
#endif
      /***************************************************/

      void setFromNode(HNode * nFrom_) {nFrom=nFrom_; }
      void setToNode(HNode * nTo_) {nTo=nTo_; }
      void setDirection(fmatvec::Vec dir) {direction=((nrm2(dir)>0)?dir/nrm2(dir):fmatvec::Vec(3, fmatvec::INIT, 0)); }
      HNode * getFromNode() { return nFrom; }
      HNode * getToNode() {return nTo; }

      virtual fmatvec::Vec getQIn(double t) = 0;
      virtual fmatvec::Vec getQOut(double t) = 0;
      virtual fmatvec::Vec getInflowFactor() = 0;
      virtual fmatvec::Vec getOutflowFactor() = 0;
      
      void updateM(double t) {M=Mlocal; }

      void init(MBSim::InitStage stage);
      void initializeUsingXML(TiXmlElement *element);
      
      MBSimControl::Signal * getSignalByPath(std::string path);
    protected:
      HNode * nFrom;
      HNode * nTo;
      fmatvec::Vec direction;
      fmatvec::SymMat Mlocal;
  };

  /*! RigidHLine */
  class RigidHLine : public HLine {
    public:
      RigidHLine(const std::string &name) : HLine(name), pressureLossGravity(0), length(0) {}
      virtual std::string getType() const { return "RigidHLine"; }
      
      void setLength(double length_) {length=length_; }
      double getLength() const {return length; }

      virtual fmatvec::Vec getQIn(double t) {return u; }
      virtual fmatvec::Vec getQOut(double t) {return -u; }
      virtual fmatvec::Vec getInflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, -1.); }
      virtual fmatvec::Vec getOutflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, 1.); }
      void calcqSize() {qSize=0; }
      void calcuSize(int j) {uSize[j]=1; }
      
      void updateh(double t);
      
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void plot(double t, double dt);
      
    protected:
      double pressureLossGravity;
      double length;
  };

}

#endif   /* ----- #ifndef _HLINE_H_  ----- */

