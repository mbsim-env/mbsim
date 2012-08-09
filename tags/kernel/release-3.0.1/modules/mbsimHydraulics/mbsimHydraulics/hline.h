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
 * Contact: markus.ms.schneider@gmail.com
 */

#ifndef  _HLINE_H_
#define  _HLINE_H_

#include "mbsim/object.h"
#include "mbsim/utils/function.h"

namespace MBSim {
  class Frame;
}

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
      HLine(const std::string &name) : MBSim::Object(name), nFrom(NULL), nTo(NULL), nFromRelative(false), nToRelative(false), direction(fmatvec::Vec(3, fmatvec::INIT, 0)), Mlocal(0), Jacobian(0,0), frameOfReference(NULL), saved_frameOfReference("") {};
      virtual std::string getType() const { return "HLine"; }

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateStateDependentVariables(double t) {};
      virtual void updateJacobians(double t, int j=0) {};
      virtual void updateInverseKineticsJacobians(double t) {};
#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual OpenMBV::Group* getOpenMBVGrp() { return 0; }
#endif
      /***************************************************/

      virtual void setFrameOfReference(MBSim::Frame *frame);
      void setFromNode(HNode * nFrom_) {nFrom=nFrom_; }
      void setToNode(HNode * nTo_) {nTo=nTo_; }
      void setDirection(fmatvec::Vec dir) {direction=((nrm2(dir)>0)?dir/nrm2(dir):fmatvec::Vec(3, fmatvec::INIT, 0)); }
      HNode * getFromNode() { return nFrom; }
      HNode * getToNode() {return nTo; }
      void setOutflowRelative(bool rel=true) {nToRelative=rel; }
      void setInflowRelative(bool rel=true) {nFromRelative=rel; }

      virtual fmatvec::Vec getQIn() = 0;
      virtual fmatvec::Vec getQOut() = 0;
      virtual fmatvec::Vec getInflowFactor() = 0;
      virtual fmatvec::Vec getOutflowFactor() = 0;
      virtual fmatvec::Mat& getJacobian() {return Jacobian; }
      
      void updateM(double t, int j=0) {M[j]=Mlocal; }

      void init(MBSim::InitStage stage);
      void initializeUsingXML(TiXmlElement *element);
      
    protected:
      HNode * nFrom;
      HNode * nTo;
      bool nFromRelative, nToRelative;
      fmatvec::Vec direction;
      fmatvec::SymMat Mlocal;
      fmatvec::Mat Jacobian;
      MBSim::Frame * frameOfReference;
    private:
      std::string saved_frameOfReference;
  };

  /*! RigidHLine */
  class RigidHLine : public HLine {
    public:
      RigidHLine(const std::string &name) : HLine(name), pressureLossGravity(0), length(0), Q(fmatvec::Vec(1)) {}
      virtual std::string getType() const { return "RigidHLine"; }
      
      void setLength(double length_) {length=length_; }
      double getLength() const {return length; }
      void addInflowDependencyOnOutflow(RigidHLine* line);
      void addInflowDependencyOnInflow(RigidHLine* line);

      virtual fmatvec::Vec getQIn() {return Q; }
      virtual fmatvec::Vec getQOut() {return -Q; }
      virtual fmatvec::Vec getInflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, -1.); }
      virtual fmatvec::Vec getOutflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, 1.); }
      void calcqSize() {qSize=0; }
      void calcuSize(int j=0) {uSize[j]=(dependency.size()?0:1); }
      fmatvec::Mat calculateJacobian(std::vector<RigidHLine*> dep_check);
      
      virtual void updateStateDependentVariables(double t);
      void updateh(double t, int j=0);
      void updateM(double t, int j=0);
      
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void plot(double t, double dt);
      
    protected:
      double pressureLossGravity;
      double length;
      fmatvec::Vec Q;
      std::vector<RigidHLine*> dependencyOnOutflow, dependencyOnInflow;
      std::vector<std::string> refDependencyOnOutflowString, refDependencyOnInflowString;
  };

  /*! ConstrainedLine */
  class ConstrainedLine : public HLine {
    public:
      ConstrainedLine(const std::string &name) : HLine(name), QFun(NULL), Q(1) {}
      virtual std::string getType() const { return "ConstrainedLine"; }
      
      void setQFunction(MBSim::Function1<double,double> * QFun_) {QFun=QFun_; }

      virtual fmatvec::Vec getQIn() {return Q; }
      virtual fmatvec::Vec getQOut() {return -Q; }
      virtual fmatvec::Vec getInflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, -1.); }
      virtual fmatvec::Vec getOutflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, 1.); }
      void calcqSize() {qSize=0; }
      void calcuSize(int j) {uSize[j]=0; }
      
      virtual void updateStateDependentVariables(double t);
      void updateh(double t, int j=0) {};
      
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      
    private:
      MBSim::Function1<double,double> * QFun;
      fmatvec::Vec Q;
  };

  /*! FluidPump */
  class FluidPump : public HLine {
    public:
      FluidPump(const std::string &name) : HLine(name), QSignal(NULL), QSignalString(""), Q(1) {}
      virtual std::string getType() const { return "FluidPump"; }
      
      void setQSignal(MBSimControl::Signal * QSignal_) {QSignal=QSignal_; }

      virtual fmatvec::Vec getQIn();
      virtual fmatvec::Vec getQOut();
      virtual fmatvec::Vec getInflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, -1.); }
      virtual fmatvec::Vec getOutflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, 1.); }
      void calcqSize() {qSize=0; }
      void calcuSize(int j) {uSize[j]=0; }
      
      void updateh(double t, int j=0) {};
      
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      
    private:
      MBSimControl::Signal * QSignal;
      std::string QSignalString;
      fmatvec::Vec Q;
  };

  /*! StatelessOrifice */
  class StatelessOrifice : public HLine {
    public:
      StatelessOrifice(const std::string &name) : HLine(name), inflowSignal(NULL), outflowSignal(NULL), openingSignal(NULL), inflowSignalString(""), outflowSignalString(""), openingSignalString(""), diameter(0), alpha(0.), calcAreaModus(0) {}
      virtual std::string getType() const { return "StatelessOrifice"; }
      
      void setInflowSignal(MBSimControl::Signal * inflowSignal_) {inflowSignal=inflowSignal_; }
      void setOutflowSignal(MBSimControl::Signal * outflowSignal_) {outflowSignal=outflowSignal_; }
      void setDiameter(double diameter_) {diameter=diameter_; }
      void setOpeningSignal(MBSimControl::Signal * openingSignal_) {openingSignal=openingSignal_; }
      void setAlpha(double alpha_) {alpha=alpha_; }
      void setCalcAreaModus(int calcAreaModus_) {calcAreaModus=calcAreaModus_; }

      virtual fmatvec::Vec getQIn();
      virtual fmatvec::Vec getQOut();
      virtual fmatvec::Vec getInflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, -1.); }
      virtual fmatvec::Vec getOutflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, 1.); }
      void calcqSize() {qSize=0; }
      void calcuSize(int j) {uSize[j]=0; }
      
      void updateh(double t, int j=0) {};
      
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void plot(double t, double dt);
      
    private:
      MBSimControl::Signal *inflowSignal, *outflowSignal, *openingSignal;
      std::string inflowSignalString, outflowSignalString, openingSignalString;
      double diameter, alpha;
      int calcAreaModus;
      fmatvec::Vec calculateQ();

      double pIn, pOut, dp, sign, opening, area, sqrt_dp;
  };

}

#endif   /* ----- #ifndef _HLINE_H_  ----- */

