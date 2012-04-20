/* Copyright (C) 2004-2010 MBSim Development Team
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

#include <config.h>
#include "mbsim/object.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Object::Object(const string &name) : Element(name), qSize(0), qInd(0) {
    uSize[0] = 0;
    uSize[1] = 0;
    hSize[0] = 0;
    hSize[1] = 0;
    uInd[0] = 0;
    uInd[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;

    setPlotFeature(state, enabled);
  } 

  Object::~Object() {}

  void Object::updatedhdz(double t) {
//    Vec h0 = h[0].copy();
//
//    updateh(t); // update with correct state
//    Vec hEnd = h[0].copy();
//
//    /**************** velocity dependent calculations ********************/
//    for(int i=0;i<uSize[0];i++) {  
//      h[0] = h0;
//
//      double ui = u(i); // save correct position
//
//      u(i) += epsroot(); // update with disturbed positions assuming same active links
//      updateStateDependentVariables(t); 
//      updateh(t);
//
//      //dhdu.col(i) = (hObject-hObjectEnd)/epsroot();
//      u(i) = ui;
//    }
//
//    /***************** position dependent calculations ********************/
//    for(int i=0;i<qSize;i++) { 
//      h[0] = h0;
//
//      double qi = q(i); // save correct position
//
//      q(i) += epsroot(); // update with disturbed positions assuming same active links
//      updateStateDependentVariables(t); 
//      updateT(t); 
//      updateJacobians(t);
//      updateh(t);
//
//      //dhdq.col(i) = (hObject-hObjectEnd)/epsroot();
//      q(i) = qi;
//    }
//
//    /******************* time dependent calculations **********************/
//    // hObject = hObject0; // set to old values
//    // h = h0;
//
//    // double t0 = t; // save correct position
//
//    // t += epsroot(); // update with disturbed positions assuming same active links
//    // updateStateDependentVariables(t); 
//    // updateT(t); 
//    // updateJacobians(t);
//    // updateh(t);
//
//    // dhdt = (hObject-hObjectEnd)/epsroot();
//    // t = t0;
//
//    /******************* back to initial state **********************/
//    updateStateDependentVariables(t); 
//    updateT(t); 
//    updateJacobians(t);
//    h[0] = hEnd;
  }

  void Object::updatedq(double t, double dt) {
    ds->getqd()(qInd,qInd+qSize-1) = ds->getT()(Index(qInd,qInd+qSize-1),Index(uInd[0],uInd[0]+uSize[0]-1))*ds->getu()(uInd[0],uInd[0]+uSize[0]-1)*dt;
  }

  void Object::updatedu(double t, double dt) {
    ds->getud(0)(uInd[0],uInd[0]+uSize[0]-1) = slvLLFac(ds->getLLM(0)(Index(hInd[0],hInd[0]+hSize[0]-1)), ds->geth(0)(hInd[0],hInd[0]+hSize[0]-1)*dt+ds->getr(0)(hInd[0],hInd[0]+hSize[0]-1));
  }

  void Object::updateud(double t, int i) {
    ds->getud(i)(uInd[i],uInd[i]+uSize[i]-1) =  slvLLFac(ds->getLLM(i)(Index(hInd[i],hInd[i]+hSize[i]-1)), ds->geth(i)(hInd[i],hInd[i]+hSize[i]-1)+ds->getr(i)(hInd[i],hInd[i]+hSize[i]-1));
  }

  void Object::updateqd(double t) {
    ds->getqd()(qInd,qInd+qSize-1) = ds->getT()(Index(qInd,qInd+qSize-1),Index(uInd[0],uInd[0]+uSize[0]-1))*ds->getu()(uInd[0],uInd[0]+uSize[0]-1);
  }

  void Object::updatezd(double t) {
    updateqd(t);
    updateud(t);
  }

  void Object::sethSize(int hSize_, int j) {
    hSize[j] = hSize_;
  }

  //int Object::gethInd(DynamicSystem* sys, int i) {
  //  return (parent == sys) ? hInd[i] : hInd[i] + parent->gethInd(sys,i);
  //}

  //int Object::getqInd(DynamicSystem* sys) {
  //  return (parent == sys) ? qInd : qInd + parent->getqInd(sys);
  //}

  //int Object::getuInd(DynamicSystem* sys, int i) {
  //  return (parent == sys) ? uInd[i] : uInd[i] + parent->getuInd(sys,i);
  //}

  void Object::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(state)==enabled) {
//        for(int i=0; i<qSize; ++i)
//          plotVector.push_back(q(i));
//        for(int i=0; i<uSize[0]; ++i)
//          plotVector.push_back(u(i));
      }
      if(getPlotFeature(stateDerivative)==enabled) {
//        for(int i=0; i<qSize; ++i)
//          plotVector.push_back(qd(i)/dt);
//        for(int i=0; i<uSize[0]; ++i)
//          plotVector.push_back(ud[0](i)/dt);
      }
//      if(getPlotFeature(rightHandSide)==enabled) {
//        for(int i=0; i<uSize[0]; ++i)
//          plotVector.push_back(h[0](i));
//        for(int i=0; i<uSize[0]; ++i)
//          plotVector.push_back(r[0](i)/dt);
//      }
      if(getPlotFeature(energy)==enabled) {
        double Ttemp = computeKineticEnergy();
        double Vtemp = computePotentialEnergy();
        plotVector.push_back(Ttemp);
        plotVector.push_back(Vtemp);
        plotVector.push_back(Ttemp + Vtemp);
      }

      Element::plot(t,dt);
    }
  }

  void Object::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Element::closePlot();
    }
  }

  void Object::init(InitStage stage) {  
    if(stage==unknownStage) {
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(state)==enabled) {
      //    for(int i=0; i<qSize; ++i)
      //      plotColumns.push_back("q("+numtostr(i)+")");
      //    for(int i=0; i<uSize[0]; ++i)
      //      plotColumns.push_back("u("+numtostr(i)+")");
        }
        if(getPlotFeature(stateDerivative)==enabled) {
      //    for(int i=0; i<qSize; ++i)
      //      plotColumns.push_back("qd("+numtostr(i)+")");
      //    for(int i=0; i<uSize[0]; ++i)
      //      plotColumns.push_back("ud("+numtostr(i)+")");
        }
        if(getPlotFeature(rightHandSide)==enabled) {
      //    for(int i=0; i<uSize[0]; ++i)
      //      plotColumns.push_back("h("+numtostr(i)+")");
      //    for(int i=0; i<getuSize(); ++i)
      //      plotColumns.push_back("r("+numtostr(i)+")");
        }
        if(getPlotFeature(energy)==enabled) {
          plotColumns.push_back("T");
          plotColumns.push_back("V");
          plotColumns.push_back("E");
        }

        Element::init(stage);
      }
    }
    else
      Element::init(stage);
  }

  void Object::initz() {
    if(q0.size())
      ds->getq()(qInd,qInd+qSize-1) = Vec(q0);
    if(u0.size())
      ds->getu()(uInd[0],uInd[0]+uSize[0]-1) = Vec(u0);
    //if (q.size()==0)
    //  q = Vec(qSize, INIT, 0);
    //if (u.size()==0)
    //  u = Vec(uSize[0], INIT, 0);
  }

  void Object::facLLM(int i) {
    ds->getLLM(i)(Index(hInd[i],hInd[i]+hSize[i]-1)) = facLL(ds->getM(i)(Index(hInd[i],hInd[i]+hSize[i]-1))); 
  }

  void Object::sethInd(int hInd_, int j) {
    hInd[j] = hInd_;
  }  

  void Object::initializeUsingXML(TiXmlElement *element) {
//    TiXmlElement *e;
//    Element::initializeUsingXML(element);
//    e=element->FirstChildElement(MBSIMNS"initialGeneralizedPosition");
//    if (e)
//      setInitialGeneralizedPosition(getVec(e));
//    e=element->FirstChildElement(MBSIMNS"initialGeneralizedVelocity");
//    if (e)
//      setInitialGeneralizedVelocity(getVec(e));
  }

  Element * Object::getByPathSearch(string path) {
    if (path.substr(0, 1)=="/") // absolut path
      if(parent)
        return parent->getByPathSearch(path);
      else
        return getByPathSearch(path.substr(1));
    else if (path.substr(0, 3)=="../") // relative path
      return parent->getByPathSearch(path.substr(3));
    else { // local path
      throw MBSimError("Unknown identifier of container");
    }
  }

  int Object::computeLevel() {
    int lOld=0;
    for(unsigned int i=0; i<dependency.size(); i++) { 
      int lNew = dependency[i]->computeLevel()+1;
      if(lNew > lOld) {
        lOld = lNew;
      }
    }
    return lOld;
  }
  
  int Object::cutDependencies() {
    int lOld=0;
    Object* buf=0;
    for(unsigned int i=0; i<dependency.size(); i++) { 
      int lNew = dependency[i]->cutDependencies()+1;
      if(lNew > lOld) {
        lOld = lNew;
        buf = dependency[i];
      }
    }
    if(dependency.size()) {
      dependency.clear();
      dependency.push_back(buf);
    }
    return lOld;
  }

  void Object::updateW0FromW1(double t) {
    //W[0] = W[1];
  }
  void Object::updateV0FromV1(double t) {
    //V[0] = V[1];
  }
  void Object::updateh0Fromh1(double t) {
    //h[0] = h[1];
  }
}

