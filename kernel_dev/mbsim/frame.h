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

#ifndef _COSY_H_
#define _COSY_H_

#include <mbsim/element.h>
#include <mbsim/interfaces.h>
#include "hdf5serie/vectorserie.h"

#ifdef HAVE_AMVIS
namespace AMVis {class Kos;}
#endif

namespace MBSim {

  class Frame;
  class Object;
#ifdef HAVE_AMVIS
  class DataInterfaceBase;
#endif

  /**
   * \brief frame on bodies used for application of e.g. links and loads
   * \author Martin Foerg
   * \date 2009-03-19 some comments (Thorsten Schindler)
   */
  class Frame : public Element {
    public:
      /**
       * \brief constructor
       * \param name of coordinate system
       */
      Frame(const string &name);

      /**
       * \brief destructor
       */
      virtual ~Frame();

      /* INHERITED INTERFACE */
      string getType() const { return "Frame"; }
      virtual void plot(double t, double dt = 1, bool top=true); 
      virtual void closePlot(); 
      virtual void initPlot(bool top=true);
      /***************************************************/

      /* GETTER / SETTER */
      ObjectInterface* getParent() { return parent; }
      void setParent(ObjectInterface* parent_) { parent = parent_; }

      int gethSize(int i=0) const { return hSize[i]; }
      int gethInd(int i=0) const { return hInd[i]; }
      void sethSize(int size, int i=0) { hSize[i] = size; }
      void sethInd(int ind, int i=0) { hInd[i] = ind; }
      
      const Vec& getPosition() const { return WrOP; }
      const Vec& getVelocity() const { return WvP; } 
      const Vec& getAngularVelocity() const { return WomegaP; }
      const SqrMat& getOrientation() const { return AWP; }
      Vec& getPosition() { return WrOP; }
      Vec& getVelocity() { return WvP; } 
      Vec& getAngularVelocity() { return WomegaP; }
      SqrMat& getOrientation() { return AWP; }

      void setPosition(const Vec &v) { WrOP = v; }
      void setVelocity(const Vec &v) { WvP = v; } 
      void setAngularVelocity(const Vec &v) { WomegaP = v; }
      void setOrientation(const SqrMat &AWP_) { AWP = AWP_; }

      void setJacobianOfTranslation(const Mat &WJP_) { WJP=WJP_; }
      void setGyroscopicAccelerationOfTranslation(const Vec &WjP_) { WjP=WjP_; }
      void setJacobianOfRotation(const Mat &WJR_) { WJR=WJR_; }
      void setGyroscopicAccelerationOfRotation(const Vec &WjR_) { WjR=WjR_; }
      const Mat& getJacobianOfTranslation() const { return WJP; }
      const Mat& getJacobianOfRotation() const { return WJR; }
      Mat& getJacobianOfTranslation() { return WJP; }
      Mat& getJacobianOfRotation() { return WJR; }
      const Vec& getGyroscopicAccelerationOfTranslation() const { return WjP; }
      const Vec& getGyroscopicAccelerationOfRotation() const { return WjR; }
      Vec& getGyroscopicAccelerationOfTranslation() { return WjP; }
      Vec& getGyroscopicAccelerationOfRotation() { return WjR; }
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

      /**
       * TODO
       */
      virtual void preinit() {}

      /**
       * \brief updates size of JACOBIANS
       */
      virtual void init();

#ifdef HAVE_AMVIS
      /**
       * TODO
       */
      void setAMVisKosSize(double size);
#endif

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
       * TODO
       */
      double *adress;

      /**
       * \brief position, velocity, angular velocity of coordinate system in inertial frame of reference
       */
      Vec WrOP, WvP, WomegaP;

      /**
       * \brief transformation matrix in inertial frame of reference
       */
      SqrMat AWP;

      /** 
       * \brief Jacobians of translation and rotation from coordinate system to inertial frame
       */
      Mat WJP, WJR;

      /**
       * translational and rotational acceleration not linear in the generalised velocity derivatives
       */
      Vec WjP, WjR;

#ifdef HAVE_AMVIS
      /**
       * TODO
       */
      AMVis::Kos *kosAMVis;
      static int kosAMVisCounter;
#endif

  };

}

#endif

