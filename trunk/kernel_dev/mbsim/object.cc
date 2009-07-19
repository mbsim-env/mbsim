/* Copyright (C) 2004-2009 MBSim Development Team
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

#include <config.h>
#include "mbsim/object.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Object::Object(const string &name) : Element(name), parent(0), frameOfReference(0), qSize(0), qInd(0) {
    uSize[0] = 0;
    uSize[1] = 0;
    hSize[0] = 0;
    hSize[1] = 0;
    uInd[0] = 0;
    uInd[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;
  } 

  Object::~Object() {}

  void Object::updatedhdq(double t) {
    Vec h0 = hObject.copy();
    for(int i=0;i<qSize;i++) {
      double qi = q(i);
      q(i) += epsroot();
      updateh(t);
      dhdq.col(i) = (hObject-h0)/epsroot();
      q(i) = qi;
    }
  }

  void Object::updatedhdu(double t) {
    Vec h0 = hObject.copy();
    for(int i=0;i<uSize[0];i++) {
      double ui = u(i);
      q(i) += epsroot();
      updateh(t);
      dhdu.col(i) = (hObject-h0)/epsroot();
      u(i) = ui;
    }
  }

  void Object::updatedhdt(double t) {
    Vec h0 = hObject.copy();
    double t0 = t;
    t += epsroot();
    updateh(t);
    dhdt = (hObject-h0)/epsroot();
    t = t0;
  }

  void Object::updatedq(double t, double dt) {
    qd = T*u*dt;
  }

  void Object::updatedu(double t, double dt) {
    ud = slvLLFac(LLM, h*dt+r);
  }

  void Object::updateud(double t) {
    ud =  slvLLFac(LLM, h+r);
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

  int Object::gethInd(DynamicSystem* sys ,int i) {
    return (parent == sys) ? hInd[i] : hInd[i] + parent->gethInd(sys,i);
  }

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
          plotVector.push_back(ud(i)/dt);
      }
      if(getPlotFeature(rightHandSide)==enabled) {
        for(int i=0; i<uSize[0]; ++i)
          plotVector.push_back(h(i));
        for(int i=0; i<uSize[0]; ++i)
          plotVector.push_back(r(i)/dt);
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

  void Object::setDynamicSystemSolver(DynamicSystemSolver* sys) {
    Element::setDynamicSystemSolver(sys);
  }

  void Object::writeq() {
    //    string fname="PREINTEG/"+getPath()+".q0.asc";  
    //    ofstream osq(fname.c_str(), ios::out);
    //    osq << q;
    //    osq.close();
  }
  void Object::readq0() {
    //    string fname="PREINTEG/"+getPath()+".q0.asc";  
    //    ifstream isq(fname.c_str());
    //    if(isq) isq >> q0;
    //    else {cout << "Object " << name << ": No Preintegration Data q0 available. Run Preintegration first." << endl; throw 50;}
    //    isq.close();
  }
  void Object::writeu() {
    //   string fname="PREINTEG/"+getPath()+".u0.asc";  
    //   ofstream osu(fname.c_str(), ios::out);
    //   osu << u;
    //   osu.close();
  }

  void Object::readu0() {
    //   string fname="PREINTEG/"+getPath()+".u0.asc";  
    //   ifstream isu(fname.c_str());
    //   if(isu) isu >> u0;
    //   else {cout << "Object " << name << ": No Preintegration Data u0 available. Run Preintegration first." << endl; throw 50;}
    //   isu.close();
  }

  void Object::writex() {
    //   string fname="PREINTEG/"+getPath()+".x0.asc";  
    //   ofstream osx(fname.c_str(), ios::out);
    //   osx << x;
    //   osx.close();
  }

  void Object::readx0() {
    //   string fname="PREINTEG/"+getPath()+".x0.asc";  
    //   ifstream isx(fname.c_str());
    //   if(isx) isx >> x0;
    //   else {cout << "Object " << name << ": No Preintegration Data x0 available. Run Preintegration first." << endl; throw 50;}
    //   isx.close();
  }

  void Object::updateqRef(const Vec &qParent) {
    q>>qParent(qInd,qInd+qSize-1);
  }

  void Object::updateqdRef(const Vec &qdParent) {
    qd>>qdParent(qInd,qInd+qSize-1);
  }

  void Object::updateuRef(const Vec &uParent) {
    u>>uParent(uInd[0],uInd[0]+uSize[0]-1);
  }

  void Object::updateudRef(const Vec &udParent) {
    ud>>udParent(uInd[0],uInd[0]+uSize[0]-1);
  }

  void Object::updatehRef(const Vec& hParent, const Vec& hObjectParent, int i) {
    h.resize()>>hParent(hInd[i],hInd[i]+hSize[i]-1);
    hObject.resize()>>hObjectParent(hInd[i],hInd[i]+hSize[i]-1);
  }

  void Object::updatedhdqRef(const Mat& dhdqParent, int i) {
    dhdq.resize()>>dhdqParent(Index(hInd[i],hInd[i]+hSize[i]-1),Index(qInd,qInd+qSize-1));
  }

  void Object::updatedhduRef(const SqrMat& dhduParent, int i) {
    dhdu.resize()>>dhduParent(Index(hInd[i],hInd[i]+hSize[i]-1),Index(uInd[0],uInd[0]+uSize[0]-1));
  }

  void Object::updatedhdtRef(const Vec& dhdtParent, int i) {
    dhdt.resize()>>dhdtParent(hInd[i],hInd[i]+hSize[i]-1);
  }

  void Object::updaterRef(const Vec& rParent) {
    r>>rParent(uInd[0],uInd[0]+uSize[0]-1);
  }

  void Object::updateTRef(const Mat &TParent) {
    T>>TParent(Index(qInd,qInd+qSize-1),Index(uInd[0],uInd[0]+uSize[0]-1));
  }

  void Object::updateMRef(const SymMat &MParent, int i) {
    M.resize()>>MParent(Index(hInd[i],hInd[i]+hSize[i]-1));
  }

  void Object::updateLLMRef(const SymMat &LLMParent, int i) {
    LLM.resize()>>LLMParent(Index(hInd[i],hInd[i]+hSize[i]-1));
  }

  void Object::init() {  
    Iu = Index(uInd[0],uInd[0]+uSize[0]-1);
    Ih = Index(hInd[0],hInd[0]+hSize[0]-1);
  }

  void Object::preinit() {  
  }

  void Object::initz() {
    q = q0;
    u = u0;
  }

  void Object::initPlot() {
    updatePlotFeatures(parent);

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

      Element::initPlot(parent);
    }
  }

  void Object::facLLM() {
    LLM = facLL(M); 
  }

  void Object::sethInd(int hInd_, int j) {
    hInd[j] = hInd_;
  }  

  string Object::getPath(char pathDelim) {
    return parent?parent->getPath()+pathDelim+name:name;
  }

  void Object::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    Element::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMNS"initialGeneralizedPosition");
    setInitialGeneralizedPosition(Vec(e->GetText()));
    e=element->FirstChildElement(MBSIMNS"initialGeneralizedVelocity");
    setInitialGeneralizedVelocity(Vec(e->GetText()));
    e=element->FirstChildElement(MBSIMNS"frameOfReference");
    setFrameOfReference(getFrameByPath(e->Attribute("ref")));
  }

  Frame *Object::getFrameByPath(string path) {
    if(path[path.length()-1]!='/') path=path+"/";
    size_t i=path.find('/');
    // absolut path
    if(i==0) {
      if(parent)
        return parent->getFrameByPath(path);
      else
        return getFrameByPath(path.substr(1));
    }
    // relative path
    string firstPart=path.substr(0, i);
    string restPart=path.substr(i+1);
    if(firstPart=="..")
      return parent->getFrameByPath(restPart);
    else
      return 0;
  }

  Contour *Object::getContourByPath(string path) {
    if(path[path.length()-1]!='/') path=path+"/";
    size_t i=path.find('/');
    // absolut path
    if(i==0) {
      if(parent)
        return parent->getContourByPath(path);
      else
        return getContourByPath(path.substr(1));
    }
    // relative path
    string firstPart=path.substr(0, i);
    string restPart=path.substr(i+1);
    if(firstPart=="..")
      return parent->getContourByPath(restPart);
    else
      return 0;
  }

}

