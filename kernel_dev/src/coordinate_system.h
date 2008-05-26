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
      int id;
      Object* parent;

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
      Object* getObject() {return parent;}
      void setObject(Object *object) {parent = object;}
      void setID(int id_) {id = id_;}
      int getID() const {return id;}

      Object* getParent() {return parent;}
      void setParent(Object *parent_) {parent = parent_;}

      virtual void init();

      const Vec& getWrOP() const {return WrOP;}
      const Vec& getWvP() const {return WvP;} 
      const Vec& getWomegaP() const {return WomegaP;}
      const SqrMat& getAWP() const {return AWP;}

      void setWrOP(const Vec &v) {WrOP = v;}
      void setWvP(const Vec &v) {WvP = v;} 
      void setWomegaP(const Vec &v) {WomegaP = v;}
      void setAWP(const SqrMat &AWP_) {AWP = AWP_;}
      
      void setWJP(const Mat &WJP_) {WJP=WJP_;}
      void setWjP(const Vec &WjP_) {WjP=WjP_;}
      void setWJR(const Mat &WJR_) {WJR=WJR_;}
      void setWjR(const Vec &WjR_) {WjR=WjR_;}
      const Mat& getWJP() const {return WJP;}
      const Mat& getWJR() const {return WJR;}
      Mat& getWJP() {return WJP;}
      Mat& getWJR() {return WJR;}
      const Vec& getWjP() const {return WjP;}
      const Vec& getWjR() const {return WjR;}
      Vec& getWjP() {return WjP;}
      Vec& getWjR() {return WjR;}

      void plot(double t, double dt=1);					// HR 03.01.07
      void initPlotFiles();
      void plotParameters();
#ifdef HAVE_AMVIS
      void setAMVisBody(AMVis::CRigidBody *AMVisBody, DataInterfaceBase *funcColor=NULL);
#endif

  };

}

#endif
