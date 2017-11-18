/* Copyright (C) 2004-2016 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _FLOATING_FRAME_LINK_H_
#define _FLOATING_FRAME_LINK_H_

#include "mbsim/links/frame_link.h"
#include "mbsim/frames/floating_relative_frame.h"
#include "mbsim/utils/index.h"

namespace MBSim {
  /** 
   * \brief floating frame link
   * \author Martin Foerg
   */
  class FloatingFrameLink : public FrameLink {
    public:
      /**
       * \brief constructor
       * \param name of link machanics
       */
      FloatingFrameLink(const std::string &name);

      void init(InitStage stage, const InitConfigSet &config) override;

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      void calcSize() override;
      void calclaSize(int j) override;
      void calcgSize(int j) override;
      void calcgdSize(int j) override;
      void calcrFactorSize(int j) override;
      void calccorrSize(int j) override;

      void initializeUsingXML(xercesc::DOMElement *element) override;

      void updateW(int i = 0) override;
      void updateh(int i = 0) override;
      void updateg() override;
      void updategd() override;

      const fmatvec::Mat3xV& evalGlobalForceDirection(int i=0) { if(updDF) updateForceDirections(); return DF; }
      const fmatvec::Mat3xV& evalGlobalMomentDirection(int i=0) { if(updDF) updateForceDirections(); return DM; }

      /** \brief The frame of reference ID for the force/moment direction vectors.
       * If ID=0 (default) the first frame, if ID=1 the second frame is used.
       */
      void setFrameOfReferenceID(Index ID) { refFrameID = ID; }

      void resetUpToDate() override;
      void updatePositions(Frame *frame) override;
      void updatePositions() override;
      void updateVelocities() override;
      void updateGeneralizedPositions() override;
      void updateGeneralizedVelocities() override;
      void updateForce() override;
      void updateMoment() override;
      void updateForceDirections() override;
      void updateR() override;

    protected:
      /**
       * \brief directions of force and moment in frame of reference
       */
      fmatvec::Mat3xV forceDir, momentDir, DF, DM;

      /**
       * \brief frame of reference the force is defined in
       */
      Frame *refFrame;
      Index refFrameID;

      /**
       * \brief own frame located in second partner with same orientation as first partner 
       */
      FloatingRelativeFrame C;

      bool updDF;
  };
}

#endif
