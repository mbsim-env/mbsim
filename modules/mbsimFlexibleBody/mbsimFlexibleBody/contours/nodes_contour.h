/* Copyright (C) 2004-2022 MBSim Development Team
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

#ifndef _NODES_CONTOUR_H_
#define _NODES_CONTOUR_H_

#include "mbsim/contours/contour.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"

namespace MBSim {
  class ContourFrame;
}

namespace OpenMBV {
  class DynamicPointSet;
}

namespace MBSimFlexibleBody {

  /*!  
   * \brief nodes contour
   * \author Martin Foerg
   */
  class NodesContour : public MBSim::Contour {
    public:
      /**
       * \brief constructor 
       * \param name of contour
       */
      NodesContour(const std::string &name="") : MBSim::Contour(name) { }

      /**
       * \brief destructor
       */
      ~NodesContour() override = default;

      void setNodeNumbers(const fmatvec::VecVI &nodes_) { nodes <<= nodes_; }
      const fmatvec::VecVI& getNodeNumbers() const { return nodes; }

      const fmatvec::Vec3& evalPosition(int i);

      void updatePositions(MBSim::ContourFrame *frame) override;
      void updateVelocities(MBSim::ContourFrame *frame) override;
      void updateAccelerations(MBSim::ContourFrame *frame) override;
      void updateJacobians(MBSim::ContourFrame *frame, int j=0) override;
      void updateGyroscopicAccelerations(MBSim::ContourFrame *frame) override;

      void plot() override;

//      virtual fmatvec::Vec3 evalWs_t(const fmatvec::Vec2 &zeta);
//      virtual fmatvec::Vec3 evalWt_t(const fmatvec::Vec2 &zeta);
//      virtual fmatvec::Vec3 evalWu_t(const fmatvec::Vec2 &zeta);
//      virtual fmatvec::Vec3 evalWv_t(const fmatvec::Vec2 &zeta);
//      virtual fmatvec::Vec3 evalWn_t(const fmatvec::Vec2 &zeta);

      MBSim::ContactKinematics * findContactPairingWith(const std::type_info &type0, const std::type_info &type1) override { return findContactPairingFlexible(type0, type1); }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      MBSim::ContourFrame* createContourFrame(const std::string &name="P") override;

    private:
      /*!
       * \brief node numbers
       */
      fmatvec::VecVI nodes;

      std::map<MBSim::ContourFrame*,int> frameMap;

      int i{0};

      std::shared_ptr<OpenMBV::DynamicPointSet> openMBVBody;
  };

}

#endif
