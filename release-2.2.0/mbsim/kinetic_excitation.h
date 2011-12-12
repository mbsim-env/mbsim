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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _KINETICEXCITATION_H_
#define _KINETICEXCITATION_H_

#include <mbsim/link_mechanics.h>
#include <mbsim/utils/function.h>

namespace MBSim {

  /**
   * \brief kinetic excitations given by time dependent functions
   * \author Markus Friedrich
   * \date 2009-08-11 some comments (Thorsten Schindler)
   */
  class KineticExcitation : public LinkMechanics {
    public:
      /**
       * \brief constructor
       * \param name of link machanics
       */
      KineticExcitation(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~KineticExcitation();

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updateh(double t);
      virtual void updateg(double) {}
      virtual void updategd(double) {}
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      void calclaSize();
      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      /***************************************************/
      
      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt = 1);
      /***************************************************/

      /** \brief Set the force excitation.
       * forceDir*func(t) is the applied force vector in space.
       * This force vector is given in the frame set by setFrameOfReference.
       */
      void setForce(fmatvec::Mat dir, Function1<fmatvec::Vec,double> *func);

      /** \brief see setForce */
      void setMoment(fmatvec::Mat dir, Function1<fmatvec::Vec,double> *func);

      /** \brief The frame of reference for the force/moment direction vectors.
       * If not given, the frame the excitation is connected to is used.
       */
      void setFrameOfReference(Frame *ref_) { refFrame=ref_; }


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

    protected:
      /**
       * \brief frame of reference the force is defined in
       */
      Frame *refFrame;

      /**
       * \brief directions of force and moment in frame of reference
       */
      fmatvec::Mat forceDir, momentDir;

      /**
       * \brief portions of the force / moment in the specific directions
       */
      Function1<fmatvec::Vec,double> *F, *M;

    private:
      std::string saved_frameOfReference, saved_ref;
  };

}

#endif /* _KINETICEXCITATION_H_ */

