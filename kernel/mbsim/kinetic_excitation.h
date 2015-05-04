/* Copyright (C) 2004-2013 MBSim Development Team
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

#include <mbsim/mechanical_link.h>
#include <mbsim/fixed_relative_frame.h>
#include <mbsim/functions/function.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#endif

namespace MBSim {

  /**
   * \brief kinetic excitations given by time dependent functions
   * \author Markus Friedrich
   * \date 2009-08-11 some comments (Thorsten Schindler)
   * \date 2013-01-09 second frame for action-reaction law (Martin Förg)
   */
  class KineticExcitation : public MechanicalLink {
    public:
      /**
       * \brief constructor
       * \param name of link machanics
       */
      KineticExcitation(const std::string &name="");

      /**
       * \brief destructor
       */
      virtual ~KineticExcitation();

      /* INHERITED INTERFACE OF LINKINTERFACE */
      void updatePositions(double t);
      void updateForceDirections(double t);
      void updateGeneralizedSingleValuedForces(double t);
      void updateh(double t, int i=0);
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      void calclaSize(int j);
      bool isActive() const { return true; }
      bool isSingleValued() const { return true; }
      bool gActiveChanged() { return false; }
      /***************************************************/
      
      /**
       * \param local force direction represented in first frame
       */
      void setForceDirection(const fmatvec::Mat3xV& fd);

      /**
       * \param local moment direction represented in first frame
       */
      void setMomentDirection(const fmatvec::Mat3xV& md);

      /** \brief Set the force excitation.
       * forceDir*func(t) is the applied force vector in space.
       * This force vector is given in the frame set by setFrameOfReference.
       */
      void setForceFunction(Function<fmatvec::VecV(double)> *func);

      /** \brief see setForce */
      void setMomentFunction(Function<fmatvec::VecV(double)> *func);

      /** \brief The frame of reference ID for the force/moment direction vectors.
       * If ID=0 the first frame, if ID=1 (default) the second frame is used.
       */
      void setFrameOfReferenceID(int ID) { refFrameID=ID; }

      using MechanicalLink::connect;

      /**
       * \param first frame to connect
       * \param second frame to connect
       */
      void connect(MBSim::Frame *frame1, MBSim::Frame *frame2);

#ifdef HAVE_OPENMBVCPPINTERFACE
      /** \brief Visualize a force arrow */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        std::vector<bool> which; which.resize(2, false);
        setOpenMBVForce(ombv.createOpenMBV());
      }

      /** \brief Visualize a moment arrow */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMoment, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toDoubleHead,referencePoint,scaleLength,scaleSize);
        setOpenMBVMoment(ombv.createOpenMBV());
      }
#endif

      void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

      virtual std::string getType() const { return "KineticExcitation"; }

      void resetUpToDate() { MechanicalLink::resetUpToDate(); C.resetUpToDate(); }

    protected:
      /**
       * \brief frame of reference the force is defined in
       */
      Frame *refFrame;
      int refFrameID;

      /**
       * \brief directions of force and moment in frame of reference
       */
      fmatvec::Mat3xV forceDir, momentDir;

      /**
       * \brief portions of the force / moment in the specific directions
       */
      Function<fmatvec::VecV(double)> *F, *M;

      /**
       * \brief own frame located in second partner with same orientation as first partner 
       */
      FixedRelativeFrame C;

    private:
      std::string saved_ref, saved_ref1, saved_ref2;
  };

}

#endif /* _KINETICEXCITATION_H_ */

