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

#ifndef  _HYDLINE_CLOSED_H_
#define  _HYDLINE_CLOSED_H_

#include "mbsim/link.h"

namespace MBSim {

  class UserFunction;
  class HydLineValve;
  class HydLineCheckvalveUnilateral;

  class HydlineClosed : public Link {
    public:

      HydlineClosed(const std::string &name, HydLineValve * line);
      ~HydlineClosed() {};
      virtual std::string getType() const { return "HydlineClosed"; }

      void calcgdSize() {gdSize=1; }
      void calcgdSizeActive() {gdSize=1; }

      void init();
      void updateW(double t);

      virtual bool isActive() const {return active; } 
      virtual bool gActiveChanged();

      virtual bool isSetValued() const {return true; }
      void calclaSize() {laSize=1; }
      void calclaSizeForActiveg() {laSize=1; }
      void calcrFactorSize() {rFactorSize=1; }

      //notwendige Funktionen aus MBSim
      virtual void updater(double t);
      virtual void updateg(double t);
      virtual void updategd(double t);
      virtual void updateWRef(const fmatvec::Mat& WRef, int i=0);
      virtual void updateVRef(const fmatvec::Mat& VRef, int i=0);
      virtual void updatehRef(const fmatvec::Vec& hRef, const fmatvec::Vec& hLinkRef, int i=0);
      virtual void updatedhdqRef(const fmatvec::Mat& dhdqRef, int i=0) {};
      virtual void updatedhduRef(const fmatvec::SqrMat& dhduRef, int i=0);
      virtual void updatedhdtRef(const fmatvec::Vec& dhdtRef, int i=0);
      virtual void updaterRef(const fmatvec::Vec& rRef);
      virtual void updatewbRef(const fmatvec::Vec& wbRef);

      void initPlot();
      void plot(double t, double dt);

    protected:
      HydLineValve * line;
      bool isActive0;
      fmatvec::Vec gdn;
      bool active;
  };

  class HydlineClosedBilateral : public HydlineClosed {
    public:
      HydlineClosedBilateral(const std::string &name, HydLineValve * line);
      ~HydlineClosedBilateral() {};
      virtual std::string getType() const { return "HydlineClosedBilateral"; }

      void updaterFactors();
      void solveImpactsFixpointSingle();
      void solveImpactsGaussSeidel();
      void checkImpactsForTermination();
  };

  class HydlineClosedUnilateral : public HydlineClosed {
    public:
      HydlineClosedUnilateral(const std::string &name, HydLineCheckvalveUnilateral * line);
      ~HydlineClosedUnilateral() {};
      virtual std::string getType() const { return "HydlineClosedUnilateral"; }

      void updaterFactors();
      void solveImpactsFixpointSingle();
      void solveImpactsGaussSeidel();
      void checkImpactsForTermination();
  };
}

#endif   /* ----- #ifndef _HYDLINE_CLOSED_H_  ----- */

