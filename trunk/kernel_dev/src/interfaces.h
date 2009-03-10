/* Copyright (C) 2004-2006  Martin Förg
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */

#ifndef _OBJECT_INTERFACE_H_
#define _OBJECT_INTERFACE_H_

namespace H5 {
  class CommonFG;
  class Group;
}

namespace MBSim {

  class Subsystem;

  class ObjectInterface {
    public:
      virtual ~ObjectInterface() {}
      virtual void updateT(double t) = 0;
      virtual void updateh(double t) = 0;
      virtual void updateM(double t) = 0;
      virtual void updateKinematics(double t) = 0;
      virtual void updateJacobians(double t) = 0;
      virtual void updatedq(double t, double dt) = 0;
      virtual void updatedu(double t, double dt) = 0;
      virtual void updateud(double t) = 0;
      virtual void updateqd(double t) = 0;
      virtual void updatezd(double t) = 0;

      virtual void sethSize(int hSize, int i=0) = 0;
      virtual int gethSize(int i=0) const = 0;
      virtual int getqSize() const = 0;
      virtual int getuSize(int i=0) const = 0;
      //virtual void calchSize() = 0;
      virtual void calcqSize() = 0;
      virtual void calcuSize(int j=0) = 0;
      virtual void setqInd(int ind) = 0;
      virtual void setuInd(int ind, int i=0) = 0;

      virtual int gethInd(Subsystem* sys, int i=0) = 0;

      virtual void updateSecondJacobians(double t) = 0;
      virtual H5::Group *getPlotGroup()=0;
      virtual PlotFeatureStatus getPlotFeature(PlotFeature fp)=0;
      virtual PlotFeatureStatus getPlotFeatureForChildren(PlotFeature fp)=0;
  };

  class LinkInterface {
    public:
      virtual ~LinkInterface() {}
      virtual void updater(double t) = 0;
      virtual void updatewb(double t) = 0;
      virtual void updateW(double t) = 0;
      virtual void updateV(double t) = 0;
      virtual void updateh(double t) = 0;
      virtual void updateg(double t) = 0;
      virtual void updategd(double t) = 0; 
      virtual void updatedx(double t, double dt) = 0;
      virtual void updatexd(double t) = 0;
      virtual void updateStopVector(double t) = 0;
      virtual void updateJacobians(double t) = 0;
  };


}

#endif
