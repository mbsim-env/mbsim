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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _ELASTIC_JOINT_H_
#define _ELASTIC_JOINT_H_

#include "mbsim/links/floating_frame_link.h"
#include "mbsim/functions/function.h"

namespace MBSim {

  /** 
   * \brief class for elastic joints
   * \author Martin Foerg
   */
  class ElasticJoint : public FloatingFrameLink {
    public:
      /**
       * \brief constructor
       * \param name
       */
      ElasticJoint(const std::string &name = "");

      /**
       * \brief destructor
       */
      ~ElasticJoint() override;

      void updateGeneralizedForces() override;

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      void updatexd() override;
      void init(InitStage stage, const InitConfigSet &config) override;
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      bool isSingleValued() const override { return true; }
      bool isActive() const override { return true; }
      bool gActiveChanged() override { return false; }
      /***************************************************/

      void setGeneralizedForceFunction(Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *func_) {
        func=func_;
        func->setParent(this);
        func->setName("GeneralizedForce");
      }

      /**
       * \param local force direction represented in first frame
       */
      void setForceDirection(const fmatvec::Mat3xV& fd);

      /**
       * \param local moment direction represented in first frame
       */
      void setMomentDirection(const fmatvec::Mat3xV& md);

      void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
      Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *func;

      /**
       * \brief translational JACOBIAN (not empty for e.g. prismatic joints)
       */
      fmatvec::Mat3xV JT;
  };

}

#endif
