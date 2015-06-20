/* Copyright (C) 2004-2014 MBSim Development Team
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

#include <hdf5serie/simpledataset.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  Object::Object(const string &name) :
      Element(name), qSize(0), qInd(0) {
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

  Object::~Object() {
  }

  void Object::updatedhdz(double t) {
    Vec h0 = h[0].copy();

    updateh(t); // update with correct state
    Vec hEnd = h[0].copy();

    /**************** velocity dependent calculations ********************/
    for (int i = 0; i < uSize[0]; i++) {
      h[0] = h0;

      double ui = u(i); // save correct position

      u(i) += epsroot(); // update with disturbed positions assuming same active links
      throw;
      //updateStateDependentVariables(t);
      updateh(t);

      //dhdu.col(i) = (hObject-hObjectEnd)/epsroot();
      u(i) = ui;
    }

    /***************** position dependent calculations ********************/
    for (int i = 0; i < qSize; i++) {
      h[0] = h0;

      double qi = q(i); // save correct position

      q(i) += epsroot(); // update with disturbed positions assuming same active links
      throw;
      //updateStateDependentVariables(t);
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
    //updateStateDependentVariables(t);
    updateT(t);
    updateJacobians(t);
    h[0] = hEnd;
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
    if (getPlotFeature(plotRecursive) == enabled) {
      if (getPlotFeature(state) == enabled) {
        for (int i = 0; i < qSize; ++i)
          plotVector.push_back(q(i));
        for (int i = 0; i < uSize[0]; ++i)
          plotVector.push_back(u(i));
      }
      if (getPlotFeature(stateDerivative) == enabled) {
        for (int i = 0; i < qSize; ++i)
          plotVector.push_back(qd(i) / dt);
        for (int i = 0; i < uSize[0]; ++i)
          plotVector.push_back(ud[0](i) / dt);
      }
      if (getPlotFeature(rightHandSide) == enabled) {
        for (int i = 0; i < uSize[0]; ++i)
          plotVector.push_back(h[0](i));
        for (int i = 0; i < uSize[0]; ++i)
          plotVector.push_back(r[0](i) / dt);
      }
      if (getPlotFeature(energy) == enabled) {
        double Ttemp = computeKineticEnergy();
        double Vtemp = computePotentialEnergy();
        plotVector.push_back(Ttemp);
        plotVector.push_back(Vtemp);
        plotVector.push_back(Ttemp + Vtemp);
      }

      Element::plot(t, dt);
    }
  }

  void Object::closePlot() {
    if (getPlotFeature(plotRecursive) == enabled) {
      Element::closePlot();
    }
  }

  //void Object::setDynamicSystemSolver(DynamicSystemSolver* sys) {
  //  Element::setDynamicSystemSolver(sys);
  //}

  void Object::updateqRef(const Vec &qParent) {
    q >> qParent(qInd, qInd + qSize - 1);
  }

  void Object::updateqdRef(const Vec &qdParent) {
    qd >> qdParent(qInd, qInd + qSize - 1);
  }

  void Object::updateuRef(const Vec &uParent) {
    u >> uParent(uInd[0], uInd[0] + uSize[0] - 1);
  }

  void Object::updateuallRef(const Vec &uParent) {
    uall >> uParent(hInd[0], hInd[0] + hSize[0] - 1);
  }

  void Object::updateudRef(const Vec &udParent, int i) {
    ud[i] >> udParent(uInd[i], uInd[i] + uSize[i] - 1);
  }

  void Object::updateudallRef(const Vec &udParent, int i) {
    udall[i] >> udParent(hInd[i], hInd[i] + hSize[i] - 1);
  }

  void Object::updatehRef(const Vec& hParent, int i) {
    h[i] >> hParent(hInd[i], hInd[i] + hSize[i] - 1);
  }

  void Object::updatedhdqRef(const Mat& dhdqParent, int i) {
    dhdq >> dhdqParent(Index(hInd[i], hInd[i] + hSize[i] - 1), Index(qInd, qInd + qSize - 1));
  }

  void Object::updatedhduRef(const SqrMat& dhduParent, int i) {
    dhdu >> dhduParent(Index(hInd[i], hInd[i] + hSize[i] - 1), Index(uInd[0], uInd[0] + uSize[0] - 1));
  }

  void Object::updatedhdtRef(const Vec& dhdtParent, int i) {
    dhdt >> dhdtParent(hInd[i], hInd[i] + hSize[i] - 1);
  }

  void Object::updaterRef(const Vec& rParent, int i) {
    r[i] >> rParent(uInd[i], uInd[i] + uSize[i] - 1);
  }

  void Object::updaterdtRef(const Vec& rdtParent, int i) {
    rdt[i] >> rdtParent(uInd[i], uInd[i] + uSize[i] - 1);
  }

  void Object::updateTRef(const Mat &TParent) {
    T >> TParent(Index(qInd, qInd + qSize - 1), Index(uInd[0], uInd[0] + uSize[0] - 1));
  }

  void Object::updateMRef(const SymMat &MParent, int i) {
    M[i] >> MParent(Index(hInd[i], hInd[i] + hSize[i] - 1));
  }

  void Object::updateLLMRef(const SymMat &LLMParent, int i) {
    LLM[i] >> LLMParent(Index(hInd[i], hInd[i] + hSize[i] - 1));
  }

  void Object::init(InitStage stage) {
    if (stage == unknownStage) {
    }
    else if (stage == plotting) {
      updatePlotFeatures();

      if (getPlotFeature(plotRecursive) == enabled) {
        if (getPlotFeature(state) == enabled) {
          for (int i = 0; i < qSize; ++i)
            plotColumns.push_back("q(" + numtostr(i) + ")");
          for (int i = 0; i < uSize[0]; ++i)
            plotColumns.push_back("u(" + numtostr(i) + ")");
        }
        if (getPlotFeature(stateDerivative) == enabled) {
          for (int i = 0; i < qSize; ++i)
            plotColumns.push_back("qd(" + numtostr(i) + ")");
          for (int i = 0; i < uSize[0]; ++i)
            plotColumns.push_back("ud(" + numtostr(i) + ")");
        }
        if (getPlotFeature(rightHandSide) == enabled) {
          for (int i = 0; i < uSize[0]; ++i)
            plotColumns.push_back("h(" + numtostr(i) + ")");
          for (int i = 0; i < getuSize(); ++i)
            plotColumns.push_back("r(" + numtostr(i) + ")");
        }
        if (getPlotFeature(energy) == enabled) {
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
    q = (q0.size() == 0) ? Vec(qSize, INIT, 0) : q0;
    u = (u0.size() == 0) ? Vec(uSize[0], INIT, 0) : u0;
  }

  void Object::writez(H5::GroupBase *group) {
    group->createChildObject<H5::SimpleDataset<vector<double> > >("q0")(q.size())->write(q);

    group->createChildObject<H5::SimpleDataset<vector<double> > >("u0")(u.size())->write(u);
  }

  void Object::readz0(H5::GroupBase *group) {
    q0.resize() = group->openChildObject<H5::SimpleDataset<vector<double> > >("q0")->read();

    u0.resize() = group->openChildObject<H5::SimpleDataset<vector<double> > >("u0")->read();
  }

  void Object::sethInd(int hInd_, int j) {
    hInd[j] = hInd_;
  }

  void Object::initializeUsingXML(DOMElement *element) {
    Element::initializeUsingXML(element);
  }

  DOMElement* Object::writeXMLFile(DOMNode *parent) {
    return Element::writeXMLFile(parent);
  }

  const Mat& Object::getT(double t) {
    if(ds->updateT()) ds->updateT(t);
    return T;
  }

  const Vec& Object::geth(double t, int i) {
    if(ds->updateh(i)) ds->updateh(t,i);
    return h[i];
  }

  const Vec& Object::getr(double t, int i) {
    if(ds->updater(i)) ds->updater(t,i);
    return r[i];
  }

  const Vec& Object::getrdt(double t, int i) {
    if(ds->updaterdt(i)) ds->updaterdt(t,i);
    return rdt[i];
  }

  const SymMat& Object::getM(double t, int i) {
    if(ds->updateM(i)) ds->updateM(t,i);
    return M[i];
  }

  const SymMat& Object::getLLM(double t, int i) {
    if(ds->updateLLM(i)) ds->updateLLM(t,i);
    return LLM[i];
  }

  void Object::updatedq(double t, double dt) {
    qd = getT(t) * u * dt;
  }

  void Object::updatedu(double t, double dt) {
    ud[0] = slvLLFac(getLLM(t), geth(t) * dt + getrdt(t));
  }

  void Object::updateud(double t, int i) {
    ud[i] = slvLLFac(getLLM(t,i), geth(t,i) + getr(t,i));
  }

  void Object::updateqd(double t) {
    qd = getT(t) * u;
  }

  const fmatvec::Vec& Object::geth(int i, bool check) const {
    assert((not check) or (not ds->updateh(i)));
    return h[i];
  }

  fmatvec::Vec& Object::geth(int i, bool check) {
    assert((not check) or (not ds->updateh(i)));
    return h[i];
  }

}

