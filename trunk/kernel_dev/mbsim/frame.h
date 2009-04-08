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

#ifndef _FRAME_H__
#define _FRAME_H__

#include "mbsim/element.h"
#include "mbsim/interfaces.h"
#include "hdf5serie/vectorserie.h"

#ifdef HAVE_AMVIS
namespace AMVis { class Kos; }
#endif
#ifdef HAVE_AMVISCPPINTERFACE
#include <amviscppinterface/frame.h>
#endif

namespace MBSim {

  /**
   * \brief cartesian stationary frame on bodies only having position and orientation
   * \author Thorsten Schindler
   * \date 2009-04-08 initial commit (Thorsten Schindler)
   * \todo AMVisEnvironmentFrame for StationaryFrame TODO
   */
  class StationaryFrame : public Element, public FrameInterface {
    public:
      /**
       * \brief constructor
       * \param name of coordinate system
       */
      StationaryFrame(const std::string &name = "dummy");

      /**
       * \brief destructor
       */
      virtual ~StationaryFrame() {}

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "StationaryFrame"; }
      virtual void plot(double t, double dt = 1); 
      virtual void closePlot(); 
      /***************************************************/

      /* INHERITED INTERFACE OF FRAMEINTERFACE */
      virtual void initPlot();
      int gethSize(int i=0) const { return 0; }
      int gethInd(int i=0) const { return 0; }
      virtual ObjectInterface* getParent() { return parent; }
      virtual void setParent(ObjectInterface* parent_) { parent = parent_; }
      virtual const fmatvec::Vec& getPosition() const { return WrOP; }
      virtual const fmatvec::SqrMat& getOrientation() const { return AWP; }
      virtual fmatvec::Vec& getPosition() { return WrOP; }
      virtual fmatvec::SqrMat& getOrientation() { return AWP; }
      virtual void setPosition(const fmatvec::Vec &v) { WrOP = v; }
      virtual void setOrientation(const fmatvec::SqrMat &AWP_) { AWP = AWP_; }
      virtual const fmatvec::Vec& getVelocity() const { return WvP; } 
      virtual const fmatvec::Vec& getAngularVelocity() const { return WomegaP; }
      virtual const fmatvec::Mat& getJacobianOfTranslation() const { return WJP; }
      virtual const fmatvec::Mat& getJacobianOfRotation() const { return WJR; }
      virtual const fmatvec::Vec& getGyroscopicAccelerationOfTranslation() const { return WjP; }
      virtual const fmatvec::Vec& getGyroscopicAccelerationOfRotation() const { return WjR; }
      virtual void preinit() {}
      virtual void init() {}
#ifdef HAVE_AMVIS
      virtual void setAMVisKosSize(double size);
#endif
#ifdef HAVE_AMVISCPPINTERFACE
      virtual void enableAMVis(double size=1, double offset=1);
#endif
      /***************************************************/

    protected:
      /**
       * \brief parent object for plot invocation
       */
      ObjectInterface* parent;

      /**
       * \brief position of coordinate system in inertial frame of reference
       */
      fmatvec::Vec WrOP;

      /**
       * \brief transformation matrix in inertial frame of reference
       */
      fmatvec::SqrMat AWP;

      /**
       * \brief position, velocity, angular velocity of coordinate system in inertial frame of reference
       */
      fmatvec::Vec WvP, WomegaP;

      /** 
       * \brief Jacobians of translation and rotation from coordinate system to inertial frame
       */
      fmatvec::Mat WJP, WJR;

      /**
       * translational and rotational acceleration not linear in the generalised velocity derivatives
       */
      fmatvec::Vec WjP, WjR;

#ifdef HAVE_AMVIS
      /**
       * TODO
       */
      AMVis::Kos *kosAMVis;
      static int kosAMVisCounter;
#endif
#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::Frame* amvisFrame;
#endif
  };

  /**
   * \brief cartesian frame on bodies used for application of e.g. links and loads
   * \author Martin Foerg
   * \date 2009-03-19 some comments (Thorsten Schindler)
   * \date 2009-04-08 stationary frame (Thorsten Schindler)
   */
  class Frame : public Element, public FrameInterface {
    public:
      /**
       * \brief constructor
       * \param name of coordinate system
       */
      Frame(const std::string &name = "dummy");

      /**
       * \brief destructor
       */
      virtual ~Frame() {}

      /* INHERITED INTERFACE ELEMENT */
      std::string getType() const { return "Frame"; }
      virtual void plot(double t, double dt = 1); 
      virtual void closePlot(); 
      /***************************************************/

      /* INHERITED INTERFACE OF FRAMEINTERFACE */
      virtual void initPlot();
      virtual int gethSize(int i=0) const { return hSize[i]; }
      virtual int gethInd(int i=0) const { return hInd[i]; }
      virtual ObjectInterface* getParent() { return parent; }
      virtual void setParent(ObjectInterface* parent_) { parent = parent_; }
      virtual const fmatvec::Vec& getPosition() const { return WrOP; }
      virtual const fmatvec::SqrMat& getOrientation() const { return AWP; }
      virtual fmatvec::Vec& getPosition() { return WrOP; }
      virtual fmatvec::SqrMat& getOrientation() { return AWP; }
      virtual void setPosition(const fmatvec::Vec &v) { WrOP = v; }
      virtual void setOrientation(const fmatvec::SqrMat &AWP_) { AWP = AWP_; }
      virtual const fmatvec::Vec& getVelocity() const { return WvP; } 
      virtual const fmatvec::Vec& getAngularVelocity() const { return WomegaP; }
      virtual const fmatvec::Mat& getJacobianOfTranslation() const { return WJP; }
      virtual const fmatvec::Mat& getJacobianOfRotation() const { return WJR; }
      virtual const fmatvec::Vec& getGyroscopicAccelerationOfTranslation() const { return WjP; }
      virtual const fmatvec::Vec& getGyroscopicAccelerationOfRotation() const { return WjR; }
      virtual void preinit() {}
      virtual void init();
#ifdef HAVE_AMVIS
      virtual void setAMVisKosSize(double size);
#endif
#ifdef HAVE_AMVISCPPINTERFACE
      virtual void enableAMVis(double size=1, double offset=1);
#endif
      /***************************************************/
      
      /* GETTER / SETTER */
      void sethSize(int size, int i=0) { hSize[i] = size; }
      void sethInd(int ind, int i=0) { hInd[i] = ind; }

      fmatvec::Vec& getVelocity() { return WvP; } 
      fmatvec::Vec& getAngularVelocity() { return WomegaP; }
      void setVelocity(const fmatvec::Vec &v) { WvP = v; } 
      void setAngularVelocity(const fmatvec::Vec &v) { WomegaP = v; }

      void setJacobianOfTranslation(const fmatvec::Mat &WJP_) { WJP=WJP_; }
      void setGyroscopicAccelerationOfTranslation(const fmatvec::Vec &WjP_) { WjP=WjP_; }
      void setJacobianOfRotation(const fmatvec::Mat &WJR_) { WJR=WJR_; }
      void setGyroscopicAccelerationOfRotation(const fmatvec::Vec &WjR_) { WjR=WjR_; }
      fmatvec::Mat& getJacobianOfTranslation() { return WJP; }
      fmatvec::Mat& getJacobianOfRotation() { return WJR; }
      fmatvec::Vec& getGyroscopicAccelerationOfTranslation() { return WjP; }
      fmatvec::Vec& getGyroscopicAccelerationOfRotation() { return WjR; }
      /***************************************************/

      /**
       * \brief updates size of JACOBIANS mainly necessary for inverse kinetics
       * \param index of right hand side
       */
      void resizeJacobians(int j);

      /**
       * \brief updates size of JACOBIANS 
       */
      void resizeJacobians();

    protected:
      /**
       * \brief parent object for plot invocation
       */
      ObjectInterface* parent;

      /**
       * \brief size and index of right hand side
       */
      int hSize[2], hInd[2];

      /**
       * \brief position of coordinate system in inertial frame of reference
       */
      fmatvec::Vec WrOP;

      /**
       * \brief transformation matrix in inertial frame of reference
       */
      fmatvec::SqrMat AWP;

      /**
       * \brief position, velocity, angular velocity of coordinate system in inertial frame of reference
       */
      fmatvec::Vec WvP, WomegaP;

      /** 
       * \brief Jacobians of translation and rotation from coordinate system to inertial frame
       */
      fmatvec::Mat WJP, WJR;

      /**
       * translational and rotational acceleration not linear in the generalised velocity derivatives
       */
      fmatvec::Vec WjP, WjR;

#ifdef HAVE_AMVIS
      /**
       * TODO
       */
      AMVis::Kos *kosAMVis;
      static int kosAMVisCounter;
#endif
#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::Frame* amvisFrame;
#endif
  };

}

#endif /* _FRAME_H_ */

