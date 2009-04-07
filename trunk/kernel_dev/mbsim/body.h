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

#ifndef _BODY_H_
#define _BODY_H_

#include <mbsim/object.h>
#ifdef HAVE_AMVISCPPINTERFACE
#include <amviscppinterface/body.h>
#endif

namespace MBSim {
  
  class Frame;
  class Contour;

  /** 
   *  \brief base class for all mechanical bodies with mass and generalised coordinates
   *  \author Martin Foerg
   *  \date 18.03.09
   */
  class Body : public Object {
    public:
      /**
       * \brief constructor
       * \param name of body
       */
      Body(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Body();

      /* INHERITED INTERFACE */
      void sethSize(int hSize_, int i=0);
      void sethInd(int hInd_, int i=0); 

      /**
       * \brief initialize body at start of simulation with respect to contours and frames
       */
      virtual void init();

      /**
       * \brief initialize body at start of simulation with respect to contours and frames TODO
       */
      virtual void preinit();

      /**
       * \param contour to add
       */
      virtual void addContour(Contour* contour);

      /**
       * \param frame to add
       */
      virtual void addFrame(Frame * frame);

      /**
       * \param name of the contour
       * \param flag for checking existence
       * \return contour
       */
      virtual Contour* getContour(const std::string &name, bool check=true);

      /**
       * \param name of the frame
       * \param flag for checking existence
       * \return frame
       */
      virtual Frame* getFrame(const std::string &name, bool check=true);

      const std::vector<Frame*>& getFrames() const { return frame; }
      const std::vector<Contour*>& getContours() const { return contour; }

      /**
       * \param frame
       * \return index of frame TODO rename
       */
      int frameIndex(const Frame *frame_) const;

      /**
       * \param contour
       * \return index of contour TODO rename
       */
      int contourIndex(const Contour *contour_) const;
      
      void setDynamicSystemSolver(DynamicSystemSolver *sys);
      
      virtual void plot(double t, double dt = 1); 
      virtual void initPlot();
      virtual void closePlot();
      virtual std::string getType() const {return "Body";}

#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::Group* getAMVisGrp() { return amvisGrp; }
#endif
    
    protected:
      /**
       * \brief vector of frames and contours
       */
      std::vector<Frame*> frame;
      std::vector<Contour*> contour;

#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::Body* amvisBody;
      AMVis::Group* amvisGrp;
#endif
  };

}

#endif

