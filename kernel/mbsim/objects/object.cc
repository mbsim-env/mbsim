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
#include "mbsim/objects/object.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"

#include <hdf5serie/simpledataset.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  const PlotFeatureEnum generalizedPosition;
  const PlotFeatureEnum generalizedVelocity;
  const PlotFeatureEnum derivativeOfGeneralizedPosition;
  const PlotFeatureEnum generalizedAcceleration;

  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, generalizedPosition)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, generalizedVelocity)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, derivativeOfGeneralizedPosition)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, generalizedAcceleration)

  Object::Object(const std::string &name) : Element(name), nq(0), nu(0), qSize(0), uSize{0,0}, hSize{0,0}, qInd(0), uInd{0,0}, hInd{0,0}, updSize(true), updq(true), updu(true), updqd(true), updud(true) {
  }

  void Object::updatedhdz() {
    THROW_MBSIMERROR("Object::updatedhdz not implemented.");
//    Vec h0 = h[0].copy();
//
//    updateh(); // update with correct state
//    Vec hEnd = h[0].copy();
//
//    /**************** velocity dependent calculations ********************/
//    for (int i = 0; i < uSize[0]; i++) {
//      h[0] = h0;
//
//      double ui = u(i); // save correct position
//
//      u(i) += epsroot; // update with disturbed positions assuming same active links
//      throw;
//      //updateStateDependentVariables(t);
//      updateh();
//
//      //dhdu.col(i) = (hObject-hObjectEnd)/epsroot;
//      u(i) = ui;
//    }
//
//    /***************** position dependent calculations ********************/
//    for (int i = 0; i < qSize; i++) {
//      h[0] = h0;
//
//      double qi = q(i); // save correct position
//
//      q(i) += epsroot; // update with disturbed positions assuming same active links
//      throw;
//
//      //dhdq.col(i) = (hObject-hObjectEnd)/epsroot;
//      q(i) = qi;
//    }
//
//    /******************* time dependent calculations **********************/
//    // hObject = hObject0; // set to old values
//    // h = h0;
//    // double t0 = t; // save correct position
//    // t += epsroot; // update with disturbed positions assuming same active links
//    // updateStateDependentVariables(t); 
//    // updateT();
//    // updateJacobians(t);
//    // updateh();
//    // dhdt = (hObject-hObjectEnd)/epsroot;
//    // t = t0;
//    /******************* back to initial state **********************/
//    h[0] = hEnd;
  }

  void Object::sethSize(int hSize_, int j) {
    hSize[j] = hSize_;
  }

  void Object::plot() {
    if (plotFeature[plotRecursive]) {
      if (plotFeature[generalizedPosition]) {
        for (int i = 0; i < evalGeneralizedPosition().size(); ++i)
          plotVector.push_back(getGeneralizedPosition()(i));
      }
      if (plotFeature[generalizedVelocity]) {
        for (int i = 0; i < evalGeneralizedVelocity().size(); ++i)
          plotVector.push_back(getGeneralizedVelocity()(i));
      }
      if (plotFeature[derivativeOfGeneralizedPosition]) {
        for (int i = 0; i < evalDerivativeOfGeneralizedPosition().size(); ++i)
          plotVector.push_back(getDerivativeOfGeneralizedPosition()(i));
      }
      if (plotFeature[generalizedAcceleration]) {
        for (int i = 0; i < evalGeneralizedAcceleration().size(); ++i)
          plotVector.push_back(getGeneralizedAcceleration()(i));
      }
      if (plotFeature[energy]) {
        double Ttemp = evalKineticEnergy();
        double Vtemp = evalPotentialEnergy();
        plotVector.push_back(Ttemp);
        plotVector.push_back(Vtemp);
        plotVector.push_back(Ttemp + Vtemp);
      }
    }
    Element::plot();
  }

  void Object::updateqRef(const Vec &qParent) {
    q >> qParent(qInd, qInd + qSize - 1);
  }

  void Object::updateqdRef(const Vec &qdParent) {
    qd >> qdParent(qInd, qInd + qSize - 1);
  }

  void Object::updatedqRef(const Vec &dqParent) {
    dq >> dqParent(qInd, qInd + qSize - 1);
  }

  void Object::updateuRef(const Vec &uParent) {
    u >> uParent(uInd[0], uInd[0] + uSize[0] - 1);
  }

  void Object::updateuallRef(const Vec &uParent) {
    uall >> uParent(hInd[0], hInd[0] + hSize[0] - 1);
  }

  void Object::updateudRef(const Vec &udParent) {
    ud >> udParent(uInd[0], uInd[0] + uSize[0] - 1);
  }

  void Object::updateduRef(const Vec &duParent) {
    du >> duParent(uInd[0], uInd[0] + uSize[0] - 1);
  }

  void Object::updateudallRef(const Vec &udParent) {
    udall >> udParent(hInd[0], hInd[0] + hSize[0] - 1);
  }

  void Object::updatehRef(const Vec& hParent, int i) {
    h[i] >> hParent(hInd[i], hInd[i] + hSize[i] - 1);
  }

  void Object::updatedhdqRef(const Mat& dhdqParent, int i) {
    dhdq >> dhdqParent(RangeV(hInd[i], hInd[i] + hSize[i] - 1), RangeV(qInd, qInd + qSize - 1));
  }

  void Object::updatedhduRef(const SqrMat& dhduParent, int i) {
    dhdu >> dhduParent(RangeV(hInd[i], hInd[i] + hSize[i] - 1), RangeV(uInd[0], uInd[0] + uSize[0] - 1));
  }

  void Object::updatedhdtRef(const Vec& dhdtParent, int i) {
    dhdt >> dhdtParent(hInd[i], hInd[i] + hSize[i] - 1);
  }

  void Object::updaterRef(const Vec& rParent, int i) {
    r[i] >> rParent(hInd[i], hInd[i] + hSize[i] - 1);
  }

  void Object::updaterdtRef(const Vec& rdtParent) {
    rdt >> rdtParent(hInd[0], hInd[0] + hSize[0] - 1);
  }

  void Object::updateTRef(const Mat &TParent) {
    T >> TParent(RangeV(qInd, qInd + qSize - 1), RangeV(uInd[0], uInd[0] + uSize[0] - 1));
  }

  void Object::updateMRef(const SymMat &MParent) {
    M >> MParent(RangeV(hInd[0], hInd[0] + hSize[0] - 1));
  }

  void Object::updateLLMRef(const SymMat &LLMParent) {
    LLM >> LLMParent(RangeV(hInd[0], hInd[0] + hSize[0] - 1));
  }

  void Object::init(InitStage stage, const InitConfigSet &config) {
    if (stage == preInit) {
      qRel.resize(getGeneralizedPositionSize());
      uRel.resize(nu);
      qdRel.resize(nq);
      udRel.resize(nu);
    }
    if (stage == plotting) {
      if (plotFeature[plotRecursive]) {
        if (plotFeature[generalizedPosition]) {
          for (int i = 0; i < qRel.size(); ++i)
            plotColumns.push_back("generalized position (" + toString(i) + ")");
        }
        if (plotFeature[generalizedVelocity]) {
          for (int i = 0; i < uRel.size(); ++i)
            plotColumns.push_back("generalized velocity (" + toString(i) + ")");
        }
        if (plotFeature[derivativeOfGeneralizedPosition]) {
          for (int i = 0; i < qdRel.size(); ++i)
            plotColumns.push_back("derivative of generalized position (" + toString(i) + ")");
        }
        if (plotFeature[generalizedAcceleration]) {
          for (int i = 0; i < udRel.size(); ++i)
            plotColumns.push_back("generalized acceleration (" + toString(i) + ")");
        }
        if (plotFeature[energy]) {
          plotColumns.push_back("kinetic energy");
          plotColumns.push_back("potential energy");
          plotColumns.push_back("total energy");
        }
      }
    }
    Element::init(stage, config);
  }

  void Object::initz() {
    if(q0() == NULL)
      q.init(0);
    else if(q0.size() == q.size())
      q = q0;
    else
      THROW_MBSIMERROR("(Object::initz): size of q0 does not match");
    if(u0() == NULL)
      u.init(0);
    else if(u0.size() == u.size())
      u = u0;
    else
      THROW_MBSIMERROR("(Object::initz): size of u0 does not match");
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

  const Mat& Object::evalT() {
    if(ds->getUpdateT()) ds->updateT();
    return T;
  }

  const Vec& Object::evalh(int i) {
    if(ds->getUpdateh(i)) ds->updateh(i);
    return h[i];
  }

  const Vec& Object::evalr(int i) {
    if(ds->getUpdater(i)) ds->updater(i);
    return r[i];
  }

  const Vec& Object::evalrdt() {
    if(ds->getUpdaterdt()) ds->updaterdt();
    return rdt;
  }

  const SymMat& Object::evalM() {
    if(ds->getUpdateM()) ds->updateM();
    return M;
  }

  const SymMat& Object::evalLLM() {
    if(ds->getUpdateLLM()) ds->updateLLM();
    return LLM;
  }

  const Vec& Object::evalqd() {
    if(ds->getUpdatezd()) ds->updatezd();
    return qd;
  }

  const Vec& Object::evalud() {
    if(ds->getUpdatezd()) ds->updatezd();
    return ud;
  }

  const Vec& Object::evaludall() {
    if(ds->getUpdatezd()) ds->updatezd();
    return udall;
  }

  void Object::updateGeneralizedPositions() {
    qRel = q;
    updq = false;
  }

  void Object::updateGeneralizedVelocities() {
    uRel = u;
    updu = false;
  }

  void Object::updateDerivativeOfGeneralizedPositions() {
    qdRel = evalqd();
    updqd = false;
  }

  void Object::updateGeneralizedAccelerations() {
    udRel = evalud();
    updud = false;
  }

  void Object::updatedq() {
    updateqd();
    dq = qd * getStepSize();
  }

  void Object::updatedu() {
    du = slvLLFac(evalLLM(), evalh() * getStepSize() + evalrdt());
  }

  void Object::updateud() {
    ud = slvLLFac(evalLLM(), evalh() + evalr());
  }

  void Object::updateqd() {
    qd = evalT() * u;
  }

  const fmatvec::Vec& Object::geth(int i, bool check) const {
    assert((not check) or (not ds->getUpdateh(i)));
    return h[i];
  }

  fmatvec::Vec& Object::geth(int i, bool check) {
    assert((not check) or (not ds->getUpdateh(i)));
    return h[i];
  }

}
