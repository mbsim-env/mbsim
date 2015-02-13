/* Copyright (C) 2004-2010 MBSim Development Team
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
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/body.h>
#endif

namespace MBSim {

  class Frame;
  class Contour;

  /** 
   *  \brief base class for all mechanical bodies with mass and generalised coordinates
   *  \author Martin Foerg
   *  \date 2009-04-06 object and body divided (Markus Schneider)
   *
   *  The following part is only a test for equation and image output for the XML documentation.
   *  A not inline equation
   *  \f[
   *    \int_a^b\sin(x)dx
   *    +5
   *  \f]
   *  And a inline equation \f$x_a+\cos(x)\f$.
   *  And a image
   *  \image html mbsim.png "The image caption"
   *  \image latex mbsim.eps "The image caption" width=4cm
   *  End of the test.
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

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      void sethSize(int hSize_, int i=0);
      void sethInd(int hInd_, int i=0); 
      /*******************************************************/ 

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt = 1); 
      virtual void closePlot();
      virtual std::string getType() const { return "Body"; }
      virtual void setDynamicSystemSolver(DynamicSystemSolver *sys);
      /*******************************************************/ 

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(InitStage stage);
      /*******************************************************/ 

      /* INTERFACE FOR DERIVED CLASSES */
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

      /**
       * \return frame of reference
       */
      virtual Frame *getFrameOfReference() { return frameOfReference; }

      /**
       * \return frame of reference
       */
      virtual const Frame *getFrameOfReference() const { return frameOfReference; }

      /**
       * \param frame of reference
       */
      virtual void setFrameOfReference(Frame *frame) { frameOfReference = frame; }
      /*******************************************************/ 

      /* GETTER / SETTER */
      const std::vector<Frame*>& getFrames() const { return frame; }
      const std::vector<Contour*>& getContours() const { return contour; }
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group* getOpenMBVGrp() { return openMBVGrp; }
      OpenMBV::Body* getOpenMBVBody() { return openMBVBody; }
#endif
      /*******************************************************/ 

      /**
       * \param frame
       * \return index of frame 
       */
      int frameIndex(const Frame *frame_) const;

      /**
       * \param contour
       * \return index of contour
       */
      int contourIndex(const Contour *contour_) const;

      virtual void initializeUsingXML(TiXmlElement *element);

      virtual Element * getByPathSearch(std::string path);

    protected:
      /**
       * \brief vector of frames and contours
       */
      std::vector<Frame*> frame;
      std::vector<Contour*> contour;

      /**
       * \brief frame of reference of the object
       */
      Frame * frameOfReference;

#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Body* openMBVBody;
      OpenMBV::Group* openMBVGrp;
#endif

    private:
      std::string saved_frameOfReference;
  };

}

#endif
