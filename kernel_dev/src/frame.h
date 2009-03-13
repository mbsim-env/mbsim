/* Copyright (C) 2004-2006  Martin Förg

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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */

#ifndef _COSY_H_
#define _COSY_H_

#include "element.h"
#include "interfaces.h"
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

  /*! Cosy on Bodies, used for application of e.g. Links and Loads
   *
   * */
  class Frame : public Element {
    protected:
      ObjectInterface* parent;

      int hSize[2], hInd[2];
      double *adress;

      Vec WrOP, WvP, WomegaP;
      SqrMat AWP;
      Mat WJP, WJR;
      Vec WjP, WjR;

#ifdef HAVE_AMVIS
      AMVis::Kos *kosAMVis;
      static int kosAMVisCounter;
#endif

    public:
      Frame(const string &name);

      ObjectInterface* getParent() {return parent;}
      void setParent(ObjectInterface* parent_) {parent = parent_;}

      int gethSize(int i=0) const {return hSize[i];}
      int gethInd(int i=0) const {return hInd[i];}
      void sethSize(int size, int i=0) {hSize[i] = size;}
      void sethInd(int ind, int i=0) {hInd[i] = ind;}

      void resizeJacobians(int j);
      void resizeJacobians();

      virtual void preinit() {}
      virtual void init();

      const Vec& getPosition() const {return WrOP;}
      const Vec& getVelocity() const {return WvP;} 
      const Vec& getAngularVelocity() const {return WomegaP;}
      const SqrMat& getOrientation() const {return AWP;}
      Vec& getPosition() {return WrOP;}
      Vec& getVelocity() {return WvP;} 
      Vec& getAngularVelocity() {return WomegaP;}
      SqrMat& getOrientation() {return AWP;}

      void setPosition(const Vec &v) {WrOP = v;}
      void setVelocity(const Vec &v) {WvP = v;} 
      void setAngularVelocity(const Vec &v) {WomegaP = v;}
      void setOrientation(const SqrMat &AWP_) {AWP = AWP_;}

      void setJacobianOfTranslation(const Mat &WJP_) {WJP=WJP_;}
      void setGyroscopicAccelerationOfTranslation(const Vec &WjP_) {WjP=WjP_;}
      void setJacobianOfRotation(const Mat &WJR_) {WJR=WJR_;}
      void setGyroscopicAccelerationOfRotation(const Vec &WjR_) {WjR=WjR_;}
      const Mat& getJacobianOfTranslation() const {return WJP;}
      const Mat& getJacobianOfRotation() const {return WJR;}
      Mat& getJacobianOfTranslation() {return WJP;}
      Mat& getJacobianOfRotation() {return WJR;}
      const Vec& getGyroscopicAccelerationOfTranslation() const {return WjP;}
      const Vec& getGyroscopicAccelerationOfRotation() const {return WjR;}
      Vec& getGyroscopicAccelerationOfTranslation() {return WjP;}
      Vec& getGyroscopicAccelerationOfRotation() {return WjR;}

      virtual void plot(double t, double dt = 1, bool top=true); 
      virtual void initPlot(bool top=true);
      virtual void closePlot(); 

      string getType() const {return "Frame";}

#ifdef HAVE_AMVIS
      void setAMVisKosSize(double size);
#endif

  };

}

#endif
