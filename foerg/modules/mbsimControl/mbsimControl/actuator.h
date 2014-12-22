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

#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include "mbsim/link_mechanics.h"
#include "mbsim/frame.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#endif

namespace MBSimControl {
  
  class Signal;

  /*!
   * \brief predefines load between two frames with additional possibility of rotation
   * \author Martin Foerg
   * \date 2009-04-06 LinkMechanics added (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \todo remove setUserfunction TODO
   * \todo visualisation TODO
   */
  class Actuator : public MBSim::LinkMechanics {
    public:
      /**
       * \brief constructor
       * \param name
       */
      Actuator(const std::string &name="");

      /**
       * \brief destructor
       */
      virtual ~Actuator() {};

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updateStateDependentVariables(double t);
      virtual void updateJacobians(double t, int j=0);
      virtual void updateh(double t, int j=0);
      virtual void updateg(double t) {}
      virtual void updategd(double t) {}
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      virtual void calclaSize(int);
      virtual bool isActive() const { return true; }
      virtual bool gActiveChanged() { return false; }
      virtual bool isSingleValued () const { return true; }
      /***************************************************/

      /* GETTER / SETTER */
      void setKOSY(int id) { KOSYID = id; assert(KOSYID >= 0); assert(KOSYID <= 2); }
      void setSignal(Signal *signal_) {signal = signal_; }
      /***************************************************/

      /**
       * \param first frame to connect
       * \param second frame to connect
       */
      void connect(MBSim::Frame *frame1, MBSim::Frame *frame2);
      
#ifdef HAVE_OPENMBVCPPINTERFACE
      /** \brief Visualize a force arrow */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVForce, MBSim::tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        MBSim::OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        std::vector<bool> which; which.resize(2, false);
        which[1]=true;
        LinkMechanics::setOpenMBVForceArrow(ombv.createOpenMBV(), which);
      }

      /** \brief Visualize a moment arrow */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMoment, MBSim::tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        MBSim::OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toDoubleHead,referencePoint,scaleLength,scaleSize);
        std::vector<bool> which; which.resize(2, false);
        which[1]=true;
        LinkMechanics::setOpenMBVMomentArrow(ombv.createOpenMBV(), which);
      }
#endif

      /**
       * \param local force direction
       */
      void setForceDirection(const fmatvec::Mat& fd);
      
      /**
       * \param local moment direction
       */
      void setMomentDirection(const fmatvec::Mat& md);

      void initializeUsingXML(xercesc::DOMElement *element);

    protected:
      /**
       * \brief indices of forces and moments
       */
      fmatvec::Index IT, IR;

      /**
       * \brief local force and moment direction
       */
      fmatvec::Mat3xV forceDir, momentDir; 

      /**
       * \brief force / moment norm function
       */
      Signal * signal;

      /**
       * \brief frame index for rotating forces
       */
      int KOSYID;

      /**
       * \brief frame of reference the force is defined in
       */
      MBSim::Frame *refFrame;

      /**
       * \brief own frame located in second partner with same orientation as first partner 
       */
      MBSim::Frame C;

    private:
      std::string saved_inputSignal, saved_ref1, saved_ref2;

  };
}

#endif

