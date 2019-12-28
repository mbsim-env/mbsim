/* Copyright (C) 2004-2016 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _SINGLE_CONTACT_OBSERVER_H__
#define _SINGLE_CONTACT_OBSERVER_H__

#include "mbsim/observers/mechanical_link_observer.h"
#include <mbsim/utils/openmbv_utils.h>
#include <openmbvcppinterface/frame.h>

namespace MBSim {

  class SingleContact;

  class SingleContactObserver : public MechanicalLinkObserver {

    friend class ContactObserver;
    friend class MaxwellContactObserver;

    protected:
      /**
       * \brief container of ContactFrames to draw
       */
      std::vector<std::shared_ptr<OpenMBV::Frame>> openMBVContactFrame;

      std::shared_ptr<OpenMBVInteractionArrow> ombvContact;
      std::shared_ptr<OpenMBVFrictionArrow> ombvFriction;

      /**
       * \brief pointer to memory of normal and friction forces to draw
       */
      std::vector<std::shared_ptr<OpenMBV::Arrow>> contactArrow, frictionArrow;

#ifndef SWIG
      double (SingleContactObserver::*evalOMBVNormalForceColorRepresentation[2])();
      double (SingleContactObserver::*evalOMBVTangentialForceColorRepresentation[3])();
#endif
      double evalNone() { return 1; }
      double evalAbsoluteNormalForce();
      double evalAbsoluteTangentialForce();
      double evalStickSlip();

    public:
      SingleContactObserver(const std::string &name="");

      void init(InitStage stage, const InitConfigSet &config) override;
      void plot() override;

      void setOpenMBVContactPoints(const std::shared_ptr<OpenMBV::Frame> &frame) { 
        openMBVContactFrame[0]=frame;
        openMBVContactFrame[1]=OpenMBV::ObjectFactory::create(openMBVContactFrame[0]);
      }

      void setOMBVNormalForce(const std::shared_ptr<OpenMBVInteractionArrow> &arrow) { ombvContact=arrow; }

      void setOMBVTangentialForce(const std::shared_ptr<OpenMBVFrictionArrow> &arrow) { ombvFriction=arrow; }
  };

}  

#endif
