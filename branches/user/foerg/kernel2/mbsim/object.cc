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
    Vec h0 = h[0].copy();

    updateh(t); // update with correct state
    Vec hEnd = h[0].copy();

    /**************** velocity dependent calculations ********************/
    for(int i=0;i<uSize[0];i++) {  
      h[0] = h0;

      double ui = u(i); // save correct position

      u(i) += epsroot(); // update with disturbed positions assuming same active links
      updateStateDependentVariables(t); 
      updateh(t);

      //dhdu.col(i) = (hObject-hObjectEnd)/epsroot();
      u(i) = ui;
    }

    /***************** position dependent calculations ********************/
    for(int i=0;i<qSize;i++) { 
      h[0] = h0;

      double qi = q(i); // save correct position

      q(i) += epsroot(); // update with disturbed positions assuming same active links
      updateStateDependentVariables(t); 
      updateT(t); 
      updateJacobians(t);
      updateh(t);

      //dhdq.col(i) = (hObject-hObjectEnd)/epsroot();
      q(i) = qi;
    }

    /******************* time dependent calculations **********************/
    // hObject = hObject0; // set to old values
    // h = h0;

    // double t0 = t; // save correct position

    // t += epsroot(); // update with disturbed positions assuming same active links
    // updateStateDependentVariables(t); 
    // updateT(t); 
    // updateJacobians(t);
    // updateh(t);

    // dhdt = (hObject-hObjectEnd)/epsroot();
    // t = t0;

    /******************* back to initial state **********************/
    updateStateDependentVariables(t); 
    updateT(t); 
    updateJacobians(t);
    h[0] = hEnd;
  }

  void Object::updatedq(double t, double dt) {
    qd = T*u*dt;
  }

  void Object::updatedu(double t, double dt) {
    ud[0] = slvLLFac(LLM[0], h[0]*dt+r[0]);
  }

  void Object::updateud(double t, int i) {
    ud[i] =  slvLLFac(LLM[i], h[i]+r[i]);
  }

  void Object::updateqd(double t) {
    qd = T*u;
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
        for(int i=0; i<qSize; ++i)
          plotVector.push_back(q(i));
        for(int i=0; i<uSize[0]; ++i)
          plotVector.push_back(u(i));
      }
      if(getPlotFeature(stateDerivative)==enabled) {
        for(int i=0; i<qSize; ++i)
          plotVector.push_back(qd(i)/dt);
        for(int i=0; i<uSize[0]; ++i)
          plotVector.push_back(ud[0](i)/dt);
      }
      if(getPlotFeature(rightHandSide)==enabled) {
        for(int i=0; i<uSize[0]; ++i)
          plotVector.push_back(h[0](i));
        for(int i=0; i<uSize[0]; ++i)
          plotVector.push_back(r[0](i)/dt);
      }
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

  //void Object::setDynamicSystemSolver(DynamicSystemSolver* sys) {
  //  Element::setDynamicSystemSolver(sys);
  //}

  void Object::updateqRef(const Vec &qParent) {
    q>>qParent(qInd,qInd+qSize-1);
  }

  void Object::updateqdRef(const Vec &qdParent) {
    qd>>qdParent(qInd,qInd+qSize-1);
  }

  void Object::updateuRef(const Vec &uParent) {
    u>>uParent(uInd[0],uInd[0]+uSize[0]-1);
  }

  void Object::updateuallRef(const Vec &uParent) {
    uall>>uParent(hInd[0],hInd[0]+hSize[0]-1);
  }

  void Object::updateudRef(const Vec &udParent, int i) {
    ud[i]>>udParent(uInd[i],uInd[i]+uSize[i]-1);
  }

  void Object::updateudallRef(const Vec &udParent, int i) {
    udall[i]>>udParent(hInd[i],hInd[i]+hSize[i]-1);
  }

  void Object::updatehRef(const Vec& hParent, int i) {
    h[i]>>hParent(hInd[i],hInd[i]+hSize[i]-1);
  }

  void Object::updateWRef(const Mat& WParent, int i) {
    W[i]>>WParent(Index(hInd[i],hInd[i]+hSize[i]-1),Index(0,WParent.cols()-1));
  }

  void Object::updateVRef(const Mat& VParent, int i) {
    V[i]>>VParent(Index(hInd[i],hInd[i]+hSize[i]-1),Index(0,VParent.cols()-1));
  }

  void Object::updatedhdqRef(const Mat& dhdqParent, int i) {
    dhdq>>dhdqParent(Index(hInd[i],hInd[i]+hSize[i]-1),Index(qInd,qInd+qSize-1));
  }

  void Object::updatedhduRef(const SqrMat& dhduParent, int i) {
    dhdu>>dhduParent(Index(hInd[i],hInd[i]+hSize[i]-1),Index(uInd[0],uInd[0]+uSize[0]-1));
  }

  void Object::updatedhdtRef(const Vec& dhdtParent, int i) {
    dhdt>>dhdtParent(hInd[i],hInd[i]+hSize[i]-1);
  }

  void Object::updaterRef(const Vec& rParent, int i) {
    r[i]>>rParent(uInd[i],uInd[i]+uSize[i]-1);
  }

  void Object::updateTRef(const Mat &TParent) {
    T>>TParent(Index(qInd,qInd+qSize-1),Index(uInd[0],uInd[0]+uSize[0]-1));
  }

  void Object::updateMRef(const SymMat &MParent, int i) {
    M[i]>>MParent(Index(hInd[i],hInd[i]+hSize[i]-1));
  }

  void Object::updateLLMRef(const SymMat &LLMParent, int i) {
    LLM[i]>>LLMParent(Index(hInd[i],hInd[i]+hSize[i]-1));
  }

  void Object::init(InitStage stage) {  
    if(stage==unknownStage) {
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(state)==enabled) {
          for(int i=0; i<qSize; ++i)
            plotColumns.push_back("q("+numtostr(i)+")");
          for(int i=0; i<uSize[0]; ++i)
            plotColumns.push_back("u("+numtostr(i)+")");
        }
        if(getPlotFeature(stateDerivative)==enabled) {
          for(int i=0; i<qSize; ++i)
            plotColumns.push_back("qd("+numtostr(i)+")");
          for(int i=0; i<uSize[0]; ++i)
            plotColumns.push_back("ud("+numtostr(i)+")");
        }
        if(getPlotFeature(rightHandSide)==enabled) {
          for(int i=0; i<uSize[0]; ++i)
            plotColumns.push_back("h("+numtostr(i)+")");
          for(int i=0; i<getuSize(); ++i)
            plotColumns.push_back("r("+numtostr(i)+")");
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
    q = (q0.size()==0)? Vec(qSize, INIT, 0) : q0;
    u = (u0.size()==0)? Vec(uSize[0], INIT, 0) : u0;
  }

  void Object::facLLM(int i) {
    LLM[i] = facLL(M[i]); 
  }

  void Object::sethInd(int hInd_, int j) {
    hInd[j] = hInd_;
  }  

  void Object::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    Element::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMNS"initialGeneralizedPosition");
    if (e)
      setInitialGeneralizedPosition(getVec(e));
    e=element->FirstChildElement(MBSIMNS"initialGeneralizedVelocity");
    if (e)
      setInitialGeneralizedVelocity(getVec(e));
  }

  TiXmlElement* Object::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Element::writeXMLFile(parent);
    if(q0.size()) {
      TiXmlElement *ele1 = new TiXmlElement( "initialGeneralizedPosition" );
      TiXmlText *text = new TiXmlText(vec2str(q0));
      ele1->LinkEndChild(text);
      ele0->LinkEndChild(ele1);
    }
    if(u0.size()) {
      TiXmlElement *ele1 = new TiXmlElement( "initialGeneralizedVelocity" );
      TiXmlText *text = new TiXmlText(vec2str(u0));
      ele1->LinkEndChild(text);
      ele0->LinkEndChild(ele1);
    }
    return ele0;
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

