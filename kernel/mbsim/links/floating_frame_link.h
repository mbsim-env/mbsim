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

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updatedhdz();
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "FloatingFrameLink"; }
      /***************************************************/

      void calclaSize(int j);
      void calcgSize(int j);
      void calcgdSize(int j);
      void calcrFactorSize(int j);
      void calccorrSize(int j);

      void initializeUsingXML(xercesc::DOMElement *element);

      void updateW(int i = 0);
      void updateh(int i = 0);
      void updateg();
      void updategd();

      /** \brief The frame of reference ID for the force/moment direction vectors.
       * If ID=0 (default) the first frame, if ID=1 the second frame is used.
       */
      void setFrameOfReferenceID(int ID) { refFrameID = ID; }

      void resetUpToDate();
      void updatePositions(Frame *frame);
      void updatePositions();
      void updateVelocities();
      void updateGeneralizedPositions();
      void updateGeneralizedVelocities();
      void updateGeneralizedForces();
      void updateForceDirections();
      void updateR();

    protected:
      /**
       * \brief directions of force and moment in frame of reference
       */
      fmatvec::Mat3xV forceDir, momentDir;

      /**
       * \brief frame of reference the force is defined in
       */
      Frame *refFrame;
      int refFrameID;

      /**
       * \brief own frame located in second partner with same orientation as first partner 
       */
      FloatingRelativeFrame C;
  };
}

#endif
