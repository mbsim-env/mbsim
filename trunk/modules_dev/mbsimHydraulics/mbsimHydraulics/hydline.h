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

  class HydNode;
  class PressureLoss;
  class PressureLossZetaVarArea;
  class UserFunction;
  class HydlineClosedBilateral;
  class HydlineClosedUnilateral;

  class HydLineAbstract : public ObjectHydraulics {
    public:
      HydLineAbstract(const std::string &name);
      ~HydLineAbstract();
      virtual std::string getType() const { return "HydLineAbstract"; }

      void setFromNode(HydNode * nFrom_);
      void setToNode(HydNode * nTo_);
      void setLength(double l_) {l=l_; }
      void setDiameter(double d_) {d=d_; }

      virtual fmatvec::Vec getQIn(double t) = 0;
      virtual fmatvec::Vec getQOut(double t) = 0;
      virtual fmatvec::Vec getInflowFactor() = 0;
      virtual fmatvec::Vec getOutflowFactor() = 0;

      void init();

    protected:
      HydNode * nFrom;
      HydNode * nTo;
      double d, l;
      double Area, rho;
  };
  
  class HydLine : public HydLineAbstract {
    public:
      HydLine(const std::string &name);
      ~HydLine();
      virtual std::string getType() const { return "HydLine"; }

      void addPressureLoss(PressureLoss * dp);
      void setQ0(double u0) {setInitialGeneralizedVelocity(u0); }

      virtual fmatvec::Vec getQIn(double t) {return u; }
      virtual fmatvec::Vec getQOut(double t) {return -u; }
      virtual fmatvec::Vec getInflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, -1.); }
      virtual fmatvec::Vec getOutflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, 1.); }

      void init();
      void calcqSize();
      void calcuSize(int j);

      void updateStateDependentVariables(double t);
      void updateh(double t);
      void updateM(double t);

      void initPlot();
      void plot(double t, double dt);

    private:
      fmatvec::Vec pLossSum;
      double zeta, MFac, zetaFac;

    protected:
      std::vector<PressureLoss*> pd;
      PressureLossZetaVarArea * pdVarArea;
  };

  class HydLineValve : public HydLine {
    public:
      HydLineValve(const std::string &name);
      virtual std::string getType() const { return "HydLineValve"; }
  };

  class HydLineValveBilateral : public HydLineValve {
    public:
      HydLineValveBilateral(const std::string &name);
      virtual std::string getType() const { return "HydLineValveBilateral"; }

      void preinit();
      void init();

      void updateStateDependentVariables(double t);

    private:
      HydlineClosedBilateral * closed;
  };

  class HydLineCheckvalveUnilateral : public HydLineValve {
    public:
      HydLineCheckvalveUnilateral(const std::string &name);
      virtual std::string getType() const { return "HydLineCheckvalveUnilateral"; }

      void preinit();

      void updateStateDependentVariables(double t);
    private:
      HydlineClosedUnilateral * closed;
  };


  class PressureLoss {
    public:
      PressureLoss(const std::string &name);
      virtual ~PressureLoss() {};
      virtual void transferLineData(double d, double l) {};

      virtual fmatvec::Vec operator()(double Q) = 0;
      virtual void initPlot(std::vector<std::string>* plotColumns);
      virtual void plot(std::vector<double>* plotVector);

    protected:
      std::string name;
      fmatvec::Vec pLoss;
  };

  class PressureLossZeta : public PressureLoss {
    public:
      PressureLossZeta(const std::string &name, double zeta);
      void transferLineData(double d, double l);

      fmatvec::Vec operator()(double Q);

    private:
      double zetaFac;
  };

  class PressureLossZetaVarArea : public PressureLoss {
    public:
      PressureLossZetaVarArea(const std::string &name, double zeta, MBSim::UserFunction * relAreaFun, double minRelArea);
      void transferLineData(double d, double l);

      void updateRelativeArea(double t);
      bool isClosed() {return closed; }

      fmatvec::Vec operator()(double Q);
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);

    private:
      MBSim::UserFunction * relAreaFun;
      double zetaFac, relArea, minRelArea;
      bool closed;
  };

  class PressureLossLaminarTubeFlow : public PressureLoss {
    public:
      PressureLossLaminarTubeFlow(const std::string &name);
      void transferLineData(double d, double l);

      fmatvec::Vec operator()(double Q);

    private:
      double lossFactor;
  };

  class PressureLossCurveFit : public PressureLoss {
    public:
      PressureLossCurveFit(const std::string &name, double dRef, double dHyd, double aPos, double bPos, double aNeg=0, double bNeg=0);
      void transferLineData(double d, double l);

      fmatvec::Vec operator()(double Q);
      void initPlot(std::vector<std::string>* plotColumns);
      void plot(std::vector<double>* plotVector);

    private:
      double Re, ReynoldsFactor;
      double aPos, bPos, aNeg, bNeg;
  };
}

#endif   /* ----- #ifndef _HYDLINE_H_  ----- */

