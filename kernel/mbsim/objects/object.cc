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
    throwError("Object::updatedhdz not implemented.");
//    Vec h0 = h[0];
//
//    updateh(); // update with correct state
//    Vec hEnd = h[0];
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
      if (plotFeature[generalizedPosition]) Element::plot(evalGeneralizedPosition());
      if (plotFeature[generalizedVelocity]) Element::plot(evalGeneralizedVelocity());
      if (plotFeature[derivativeOfGeneralizedPosition]) Element::plot(evalDerivativeOfGeneralizedPosition());
      if (plotFeature[generalizedAcceleration]) Element::plot(evalGeneralizedAcceleration());
    }
    Element::plot();
  }

  void Object::updateqRef(Vec &qParent) {
    q.ref(qParent, RangeV(qInd, qInd + qSize - 1));
  }

  void Object::updateqdRef(Vec &qdParent) {
    qd.ref(qdParent, RangeV(qInd, qInd + qSize - 1));
  }

  void Object::updatedqRef(Vec &dqParent) {
    dq.ref(dqParent, RangeV(qInd, qInd + qSize - 1));
  }

  void Object::updateuRef(Vec &uParent) {
    u.ref(uParent, RangeV(uInd[0], uInd[0] + uSize[0] - 1));
  }

  void Object::updateuallRef(Vec &uParent) {
    uall.ref(uParent, RangeV(hInd[0], hInd[0] + hSize[0] - 1));
  }

  void Object::updateudRef(Vec &udParent) {
    ud.ref(udParent, RangeV(uInd[0], uInd[0] + uSize[0] - 1));
  }

  void Object::updateduRef(Vec &duParent) {
    du.ref(duParent, RangeV(uInd[0], uInd[0] + uSize[0] - 1));
  }

  void Object::updateudallRef(Vec &udParent) {
    udall.ref(udParent, RangeV(hInd[0], hInd[0] + hSize[0] - 1));
  }

  void Object::updatehRef(Vec& hParent, int i) {
    h[i].ref(hParent, RangeV(hInd[i], hInd[i] + hSize[i] - 1));
  }

  void Object::updatedhdqRef(Mat& dhdqParent, int i) {
    dhdq.ref(dhdqParent, RangeV(hInd[i], hInd[i] + hSize[i] - 1), RangeV(qInd, qInd + qSize - 1));
  }

  void Object::updatedhduRef(SqrMat& dhduParent, int i) {
    dhdu.ref(dhduParent, RangeV(hInd[i], hInd[i] + hSize[i] - 1), RangeV(uInd[0], uInd[0] + uSize[0] - 1));
  }

  void Object::updatedhdtRef(Vec& dhdtParent, int i) {
    dhdt.ref(dhdtParent, RangeV(hInd[i], hInd[i] + hSize[i] - 1));
  }

  void Object::updaterRef(Vec& rParent, int i) {
    r[i].ref(rParent, RangeV(hInd[i], hInd[i] + hSize[i] - 1));
  }

  void Object::updaterdtRef(Vec& rdtParent) {
    rdt.ref(rdtParent, RangeV(hInd[0], hInd[0] + hSize[0] - 1));
  }

  void Object::updateTRef(Mat &TParent) {
    T.ref(TParent, RangeV(qInd, qInd + qSize - 1), RangeV(uInd[0], uInd[0] + uSize[0] - 1));
  }

  void Object::updateMRef(SymMat &MParent) {
    M.ref(MParent, RangeV(hInd[0], hInd[0] + hSize[0] - 1));
  }

  void Object::updateLLMRef(SymMat &LLMParent) {
    LLM.ref(LLMParent, RangeV(hInd[0], hInd[0] + hSize[0] - 1));
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
        if (plotFeature[generalizedPosition]) addToPlot("generalized position",qRel.size());
        if (plotFeature[generalizedVelocity]) addToPlot("generalized velocity",uRel.size());
        if (plotFeature[derivativeOfGeneralizedPosition]) addToPlot("derivative of generalized position",qdRel.size());
        if (plotFeature[generalizedAcceleration]) addToPlot("generalized acceleration",udRel.size());
      }
    }
    Element::init(stage, config);
  }

  void Object::initz() {
    if(not q0())
      q.init(0);
    else if(q0.size() == q.size())
      q = q0;
    else
      throwError("(Object::initz): size of q0 does not match, must be " + to_string(q.size()));
    if(not u0())
      u.init(0);
    else if(u0.size() == u.size())
      u = u0;
    else
      throwError("(Object::initz): size of u0 does not match, must be " + to_string(u.size()));
  }

  void Object::writez(H5::GroupBase *group) {
    group->createChildObject<H5::SimpleDataset<vector<double>>>("q0")(q.size())->write((vector<double>)q);

    group->createChildObject<H5::SimpleDataset<vector<double>>>("u0")(u.size())->write((vector<double>)u);
  }

  void Object::readz0(H5::GroupBase *group) {
    q0 <<= Vec(group->openChildObject<H5::SimpleDataset<vector<double>>>("q0")->read());

    u0 <<= Vec(group->openChildObject<H5::SimpleDataset<vector<double>>>("u0")->read());
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

  Vec& Object::getud(bool check) {
    assert((not check) or (not ds->getUpdatezd()));
    return ud;
  }

  Vec& Object::getudall(bool check) {
    assert((not check) or (not ds->getUpdatezd()));
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

  const SymMat& Object::getM(bool check) const {
    assert((not check) or (not ds->getUpdateM()));
    return M;
  }

  SymMat& Object::getM(bool check) {
    assert((not check) or (not ds->getUpdateM()));
    return M;
  }

  const Vec& Object::geth(int i, bool check) const {
    assert((not check) or (not ds->getUpdateh(i)));
    return h[i];
  }

  Vec& Object::geth(int i, bool check) {
    assert((not check) or (not ds->getUpdateh(i)));
    return h[i];
  }

  void Object::createPlotGroup() {
    plotGroup=parent->getObjectsPlotGroup()->createChildObject<H5::Group>(name)();
    plotGroup->createChildAttribute<H5::SimpleAttribute<string>>("Description")()->write("Object of class: "+boost::core::demangle(typeid(*this).name()));
    plotColumns.insert(plotColumns.begin(), "time");
  }

}
