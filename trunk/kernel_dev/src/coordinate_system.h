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

#ifdef HAVE_AMVIS
namespace AMVis {class CRigidBody;}
#endif

namespace MBSim {
 
  class Object;
#ifdef HAVE_AMVIS
  class DataInterfaceBase;
#endif

  /*! Cosy on Bodies, used for application of e.g. Links and Loads
   *
   * */
  class CoordinateSystem : public Element {
    protected:
    //  Object* parent;
      int hSize, hInd;

      Vec WrOP, WvP, WomegaP;
      SqrMat AWP;
      Mat WJP, WJR;
      Vec WjP, WjR;

#ifdef HAVE_AMVIS
      AMVis::CRigidBody *bodyAMVis;
      DataInterfaceBase* bodyAMVisUserFunctionColor;
#endif

    public:
      CoordinateSystem(const string &name);

      //string getFullName() const; 

    //  Object* getObject() {return parent;}
    //  void setObject(Object *object) {parent = object;}

    //  Object* getParent() {return parent;}
    //  void setParent(Object *parent_) {parent = parent_;}

      int gethSize() const {return hSize;}
      int gethInd() const {return hInd;}
      void sethSize(int size) {hSize = size;}
      void sethInd(int ind) {hInd = ind;}

      virtual void init();

      const Vec& getPosition() const {return WrOP;}
      const Vec& getVelocity() const {return WvP;} 
      const Vec& getAngularVelocity() const {return WomegaP;}
      const SqrMat& getOrientation() const {return AWP;}

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

      void plot(double t, double dt=1);					// HR 03.01.07
      void initPlotFiles();
      string getType() const {return "CoordinateSystem";}

#ifdef HAVE_AMVIS
      void setAMVisBody(AMVis::CRigidBody *AMVisBody, DataInterfaceBase *funcColor=NULL);
#endif

  };

}

#endif
