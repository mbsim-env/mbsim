/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#ifndef _FLEXIBLE_BODY_2S_13_DISK_H_
#define _FLEXIBLE_BODY_2S_13_DISK_H_

#include "mbsimFlexibleBody/flexible_body/flexible_body_2s_13.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_2s_13_disk.h"

namespace MBSimFlexibleBody {

  /*! 
   * \brief plate according to Reissner-Mindlin with axial moving frame of reference
   * \author Roland Zander
   * \author Thorsten Schindler
   * \author Kilian Grundl
   * \author Raphael Missel
   * \author Adrian Staszkiewicz
   * \date 2009-05-14 initial commit (Schindler / Grundl / Missel)
   * \date 2009-08-15 contour / visualisation (Schindler / Grundl / Missel)
   * \date 2010-04-21 fixed position and velocity at surface node (Schindler)
   * \todo gravity TODO
   */
  class FlexibleBody2s13Disk : public FlexibleBody2s13 {
    public:
      /**
       * \brief constructor
       * \param name of body
       */
      FlexibleBody2s13Disk(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~FlexibleBody2s13Disk() { }

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      virtual void BuildElements(double t);
      virtual void GlobalVectorContribution(int CurrentElement, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
      virtual void GlobalMatrixContribution(int CurrentElement, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat);
      virtual void GlobalMatrixContribution(int CurrentElement, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat);

      fmatvec::Vec3 getPosition(double t);
      fmatvec::SqrMat3 getOrientation(double t);

      virtual void updatePositions(double t, Frame2s* frame);
      virtual void updateVelocities(double t, Frame2s* frame);
      virtual void updateAccelerations(double t, Frame2s* frame);
      virtual void updateJacobians(double t, Frame2s* frame, int j=0);
      virtual void updateGyroscopicAccelerations(double t, Frame2s* frame);

      virtual void updatePositions(double t, NodeFrame* frame);
      virtual void updateVelocities(double t, NodeFrame* frame);
      virtual void updateAccelerations(double t, NodeFrame* frame);
      virtual void updateJacobians(double t, NodeFrame* frame, int j=0);
      virtual void updateGyroscopicAccelerations(double t, NodeFrame* frame);

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "FlexibleBody2s13Disk"; }
      /***************************************************/

      /* INHERITED INTERFACE OF FLEXIBLEBODY2s13 */
      virtual fmatvec::Vec transformCW(double t, const fmatvec::Vec& WrPoint);
      /***************************************************/

    protected:
      /* INHERITED INTERFACE OF FLEXIBLEBODY2s13 */
      virtual void initMatrices();
      virtual void updateAG(double t);
      /***************************************************/

  };

}

#endif /* _FLEXIBLE_BODY_2S_13_DISK_H_ */
