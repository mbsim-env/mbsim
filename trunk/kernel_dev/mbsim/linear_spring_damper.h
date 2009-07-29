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

#ifndef _LINEARSPRINGDAMPER_H_
#define _LINEARSPRINGDAMPER_H_

#include "mbsim/link_mechanics.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/coilspring.h>
#endif

namespace MBSim {

  /**
   * \brief linear spring damper link
   * \author Martin Foerg
   * \date 2009-07-07 some comments (Thorsten Schindler)
   * \date 2009-07-27 implicit integration (Thorsten Schindler)
   * \todo separate force law after having cleaned function concept
   */
  class LinearSpringDamper : public LinkMechanics {
    public:
      /** 
       * \brief constructor
       * \param name of spring damper
       */
      LinearSpringDamper(const std::string &name);

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updateh(double t); 
      virtual void updateg(double t);
      virtual void updategd(double t); 
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void init();
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "LinearSpringDamper"; }
      virtual void plot(double t,double dt=1); 
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      virtual bool isActive() const {return true;}
      virtual bool gActiveChanged() {return false;}
      /***************************************************/

      /* INHERITED INTERFACE OF LINKMECHANICS */
      virtual void initPlot();
      virtual void connect(Frame *frame1, Frame* frame2);
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setUnloadedLength(double l0_) {l0=l0_;}
      void setStiffnessCoefficient(double c) {cT = c;}
      void setDampingCoefficient(double d) {dT = d;}
#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpring(OpenMBV::CoilSpring *spring_) {coilspringOpenMBV=spring_;}
#endif
      /***************************************************/

    protected:
      /*
       * \brief unloaded length, stiffness, damping
       */
      double l0, cT, dT;

      /**
       * \brief loaded length
       */
      double dist;
      
      /**
       * \brief direction of force
       */
      fmatvec::Vec forceDir;

#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::CoilSpring *coilspringOpenMBV;
#endif
  };

}

#endif /* _LINEARSPRINGDAMPER_H_ */

