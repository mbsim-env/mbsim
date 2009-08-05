/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _SPRINGDAMPER_H_
#define _SPRINGDAMPER_H_

#include "mbsim/link_mechanics.h"
#include "mbsim/utils/function.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/coilspring.h>
#endif

namespace MBSim {

  class SpringDamper : public LinkMechanics {
    protected:
      double dist;
      fmatvec::Vec n;
      Function2<double,double,double> *func;
      Frame *refFrame;
      fmatvec::Vec forceDir;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::CoilSpring *coilspringOpenMBV;
#endif
    public:
      SpringDamper(const std::string &name);
      void updateh(double);
      void updateg(double);
      void updategd(double);
      void connect(Frame *frame1, Frame* frame2);
      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      std::string getType() const { return "SpringDamper"; }
      void init();
      void setForceFunction(Function2<double,double,double> *func_) { func=func_; }
      void setProjectionDirection(Frame *frame, fmatvec::Vec dir) { refFrame=frame; forceDir=dir; }
      void initPlot();
      void plot(double t, double dt=1);
      void initializeUsingXML(TiXmlElement *element);
#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpring(OpenMBV::CoilSpring *spring_) {coilspringOpenMBV=spring_;}
#endif
  };

}

#endif /* _LINEARSPRINGDAMPER_H_ */

