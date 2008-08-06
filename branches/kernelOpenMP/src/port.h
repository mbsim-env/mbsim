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

#ifndef _PORT_H_
#define _PORT_H_

#include "element.h"

#ifdef HAVE_AMVIS
namespace AMVis {class CRigidBody;}
#endif

namespace MBSim {
 
  class Object;
#ifdef HAVE_AMVIS
  class DataInterfaceBase;
#endif

  /*! Port on Bodies, used for application of e.g. Links and Loads
   *
   * */
  class Port : public Element {
    protected:
      int id;
      Object* parent;

      Vec WrOP, WvP, WomegaP;
      SqrMat AWP;

#ifdef HAVE_AMVIS
      AMVis::CRigidBody *bodyAMVis;
      DataInterfaceBase* bodyAMVisUserFunctionColor;
      int AMVisInstance;
#endif

    public:
      Port(const string &name);
      Object* getObject() {return parent;}
      void setObject(Object *object) {parent = object;}
      void setID(int id_) {id = id_;}
      int getID() const {return id;}

      virtual void init();

      const Vec& getWrOP() const {return WrOP;}
      const Vec& getWvP() const {return WvP;} 
      const Vec& getWomegaP() const {return WomegaP;}
      const SqrMat& getAWP() const {return AWP;}

      void setWrOP(const Vec &v) {WrOP = v;}
      void setWvP(const Vec &v) {WvP = v;} 
      void setWomegaP(const Vec &v) {WomegaP = v;}
      void setAWP(const SqrMat &AWP_) {AWP = AWP_;}

      void plot(double t, double dt=1);					// HR 03.01.07
      void initPlotFiles();
      void plotParameters();
#ifdef HAVE_AMVIS
      void setAMVisBody(AMVis::CRigidBody *AMVisBody, DataInterfaceBase *funcColor=NULL, int instance=0);
#endif

  };

}

#endif
