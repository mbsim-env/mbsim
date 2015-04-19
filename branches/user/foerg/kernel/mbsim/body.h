/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _BODY_H_
#define _BODY_H_

#include <mbsim/object.h>
#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class Body;
}
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

      void setInitialGeneralizedPosition(const fmatvec::Vec &q0_) { q0 = q0_; }
      void setInitialGeneralizedVelocity(const fmatvec::Vec &u0_) { u0 = u0_; }
      void setInitialGeneralizedPosition(double q0_) { q0 = fmatvec::Vec(1,fmatvec::INIT,q0_); }
      void setInitialGeneralizedVelocity(double u0_) { u0 = fmatvec::Vec(1,fmatvec::INIT,u0_); }

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
       * \param name of the contour
       * \param flag for checking existence
       * \return contour
       */
      virtual Contour* getContour(const std::string &name, bool check=true) const;

      /**
       * \param name of the frame
       * \param flag for checking existence
       * \return frame
       */
      virtual Frame* getFrame(const std::string &name, bool check=true) const;

      /**
       * \return frame of reference
       */
      virtual Frame *getFrameOfReference() { return R; }

      /**
       * \return frame of reference
       */
      virtual const Frame *getFrameOfReference() const { return R; }

      /**
       * \param frame of reference
       */
      virtual void setFrameOfReference(Frame *frame) { R = frame; }
      /*******************************************************/ 

      /* GETTER / SETTER */
      const std::vector<Frame*>& getFrames() const { return frame; }
      const std::vector<Contour*>& getContours() const { return contour; }
#ifdef HAVE_OPENMBVCPPINTERFACE
      boost::shared_ptr<OpenMBV::Group> getOpenMBVGrp() { return openMBVGrp; }
      boost::shared_ptr<OpenMBV::Body>& getOpenMBVBody() { return openMBVBody; }
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

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

      virtual Element * getChildByContainerAndName(const std::string &container, const std::string &name) const;

      fmatvec::Mat3xV& getPJT(int i=0) {return PJT[i];}
      fmatvec::Mat3xV& getPJR(int i=0) {return PJR[i];}
      int getuRelSize(int i=0) const {return nu[i];}

      void resetUpToDate();

    protected:
      /**
       * \param frame to add
       */
      virtual void addFrame(Frame * frame);

      /**
       * \param contour to add
       */
      virtual void addContour(Contour* contour);

      /**
       * \brief vector of frames and contours
       */
      std::vector<Frame*> frame;
      std::vector<Contour*> contour;

      /**
       * \brief frame of reference of the object
       */
      Frame *R;

      /**
       * JACOBIAN of translation, rotation and their derivatives in parent system
       */
      fmatvec::Mat3xV PJT[2], PJR[2];

      int nu[2], nq;

#ifdef HAVE_OPENMBVCPPINTERFACE
      boost::shared_ptr<OpenMBV::Body> openMBVBody;
      boost::shared_ptr<OpenMBV::Group> openMBVGrp;
#endif

    private:
      std::string saved_frameOfReference;
  };

}

#endif

