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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _KINETICEXCITATION_H_
#define _KINETICEXCITATION_H_

#include <mbsim/link_mechanics.h>
#include <mbsim/utils/function.h>
#include <mbsimtinyxml/tinyxml-src/tinyxml.h>

namespace MBSim {

  class DataInterfaceBase;

  class KineticExcitation : public LinkMechanics {
    protected:
      Frame *refFrame;
      fmatvec::Mat forceDir, momentDir;
      Function1<fmatvec::Vec,double> *F, *M;
    public: 
      KineticExcitation(const std::string &name);
      virtual ~KineticExcitation();
      void updateh(double t);
      void calclaSize();
      void init();

      /** \brief Set the force excitation.
       * dir*func(t) is the applied force vector in R3.
       * This force vector is given in the frame set by setFrameOfReference.
       */
      void setForce(fmatvec::Mat dir, Function1<fmatvec::Vec,double> *func);

      /** see setForce */
      void setMoment(fmatvec::Mat dir, Function1<fmatvec::Vec,double> *func);

      /** \brief The frame of reference for the force/moment direction vectors.
       * If not given, the frame the excitation is connected to is used.
       */
      void setFrameOfReference(Frame *ref_) { refFrame=ref_; }

      void updateg(double) {}
      void updategd(double) {}
      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }

#ifdef HAVE_OPENMBVCPPINTERFACE
      /** \brief Visualize a force arrow acting on the frame */
      void setOpenMBVForceArrow(OpenMBV::Arrow *arrow) {
        std::vector<bool> which; which.resize(1, true);
        LinkMechanics::setOpenMBVForceArrow(arrow, which);
      }

      /** \brief Visualize a moment arrow acting on the frame */
      void setOpenMBVMomentArrow(OpenMBV::Arrow *arrow) {
        std::vector<bool> which; which.resize(1, true);
        LinkMechanics::setOpenMBVMomentArrow(arrow, which);
      }
#endif

      void initializeUsingXML(TiXmlElement *element);
  };

}

#endif

