/* Copyright (C) 2004-2014 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/links/link.h"
#include "mbsim/dynamic_system_solver.h"

#include <hdf5serie/simpledataset.h>

using namespace fmatvec;
using namespace std;

namespace MBSim {

  const PlotFeatureEnum generalizedRelativePosition;
  const PlotFeatureEnum generalizedRelativeVelocity;
  const PlotFeatureEnum generalizedForce;

  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, generalizedRelativePosition)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, generalizedRelativeVelocity)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, generalizedForce)

  Link::Link(const string &name) : Element(name), ng(0), ngd(0), nla(0), xSize(0), xInd(0), svSize(0), svInd(0), LinkStatusSize(0), LinkStatusInd(0), LinkStatusRegSize(0), LinkStatusRegInd(0), isSize(0), isInd(0), gSize(0), gInd(0), gdSize(0), gdInd(0), laSize(0), laInd(0), bSize(0), bInd(0), gTol(1e-8), gdTol(1e-10), gddTol(1e-12), laTol(1e-12), LaTol(1e-10), gCorr(1e-14), gdCorr(2e-16), rFactorSize(0), rFactorInd(0), rMax(1.), corrSize(0), corrInd(0), updSize(true), updrrel(true), updvrel(true), updla(true) {
  }

  void Link::plot() {
    if(plotFeature[plotRecursive]) {
      if(plotFeature[generalizedRelativePosition]) Element::plot(evalGeneralizedRelativePosition());
      if(plotFeature[generalizedRelativeVelocity]) Element::plot(evalGeneralizedRelativeVelocity());
      if(plotFeature[generalizedForce]) Element::plot(evalGeneralizedForce());
    }
    Element::plot();
  }

  void Link::updatedx() {
    updatexd();
    dx = xd * getStepSize();
  }

  const Vec& Link::getxd(bool check) const {
    assert((not check) or (not ds->getUpdatezd()));
    return xd;
  }

  Vec& Link::getxd(bool check) {
    assert((not check) or (not ds->getUpdatezd()));
    return xd;
  }

  void Link::updatewbRef(Vec& wbParent) {
    wb.ref(wbParent, RangeV(laInd,laInd+laSize-1));
  }

  void Link::updatexRef(Vec &xParent) {
    x.ref(xParent, RangeV(xInd,xInd+xSize-1));
  } 

  void Link::updatexdRef(Vec &xdParent) {
    xd.ref(xdParent, RangeV(xInd,xInd+xSize-1));
  } 

  void Link::updatedxRef(Vec &dxParent) {
    dx.ref(dxParent, RangeV(xInd,xInd+xSize-1));
  }

  void Link::updatelaRef(Vec& laParent) {
    la.ref(laParent, RangeV(laInd,laInd+laSize-1));
  }

  void Link::updateLaRef(Vec& LaParent) {
    La.ref(LaParent, RangeV(laInd,laInd+laSize-1));
  }

  void Link::deletelaRef() {
    la.resize(la.size(), NONINIT);
  }

  void Link::updateInternalStateRef(Vec& curisParent, Vec& nextisParent) {
    curis.ref(curisParent, RangeV(isInd,isInd+isSize-1));
    nextis.ref(nextisParent, RangeV(isInd,isInd+isSize-1));
  }

  void Link::updategRef(Vec& gParent) {
    g.ref(gParent, RangeV(gInd,gInd+gSize-1));
  }

  void Link::updategdRef(Vec& gdParent) {
    gd.ref(gdParent, RangeV(gdInd,gdInd+gdSize-1));
  }

  void Link::updateresRef(Vec& resParent) {
    res.ref(resParent, RangeV(laInd,laInd+laSize-1));
  }

  void Link::updaterFactorRef(Vec& rFactorParent) {
    rFactor.ref(rFactorParent, RangeV(rFactorInd,rFactorInd+rFactorSize-1));
  }

  void Link::updatesvRef(Vec &svParent) {
    sv.ref(svParent, RangeV(svInd,svInd+svSize-1));
  }

  void Link::updatejsvRef(VecInt &jsvParent) {
    jsv.ref(jsvParent, RangeV(svInd,svInd+svSize-1));
  }
   
  void Link::updateLinkStatusRef(VecInt &LinkStatusParent) {
    LinkStatus.ref(LinkStatusParent, RangeV(LinkStatusInd,LinkStatusInd+LinkStatusSize-1));
  }

  void Link::updateLinkStatusRegRef(VecInt &LinkStatusRegParent) {
    LinkStatusReg.ref(LinkStatusRegParent, RangeV(LinkStatusRegInd,LinkStatusRegInd+LinkStatusRegSize-1));
  }

  void Link::updatebRef(Mat &bParent) {
    RangeV J = RangeV(laInd,laInd+laSize-1);
    RangeV I = RangeV(bInd,bInd+bSize-1);
    b.ref(bParent,I,J);
  } 

  void Link::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      rrel.resize(getGeneralizedRelativePositionSize());
      vrel.resize(ngd);
      lambda.resize(nla);
    }
    else if(stage==plotting) {
      if(plotFeature[plotRecursive]) {
        if(plotFeature[generalizedRelativePosition]) addToPlot("generalized relative position",rrel.size());
        if(plotFeature[generalizedRelativeVelocity]) addToPlot("generalized relative velocity",vrel.size());
        if(plotFeature[generalizedForce]) addToPlot("generalized force",lambda.size());
      }
    }
    else if(stage==unknownStage)
      rFactorUnsure.resize(rFactorSize);
    Element::init(stage, config);
  }

  void Link::initz() {
    if(x0() == NULL)
      x.init(0);
    else if(x0.size() == x.size())
      x = x0;
    else
      throwError("(Constraint::initz): size of x0 does not match, must be " + to_string(x.size()));
  }

  void Link::writez(H5::GroupBase *group) {
    group->createChildObject<H5::SimpleDataset<vector<double>>>("x0")(x.size())->write((vector<double>)x);
  }

  void Link::readz0(H5::GroupBase *group) {
    x0 <<= Vec(group->openChildObject<H5::SimpleDataset<vector<double>>>("x0")->read());
  }

  void Link::savela() {
    la0 <<= la;
  }

  void Link::initla() {
    if(la0.size() == la.size()) // TODO check if initialising to 0 is better if contact was inactive before
      la = la0;
    else
      la.init(0);
  }

  void Link::saveLa() {
    La0 <<= La;
  }

  void Link::initLa() {
    if(La0.size() == La.size()) // TODO check if initialising to 0 is better if contact was inactive before
      La = La0;
    else
      La.init(0);
  }

  void Link::decreaserFactors() {
    for(int i=0; i<rFactor.size(); i++)
      if(rFactorUnsure(i))
        rFactor(i) *= 0.9;
  }

  void Link::updatecorrRef(fmatvec::Vec &ref) {
    corr.ref(ref, RangeV(corrInd,corrInd+corrSize-1));
  }

  const Vec& Link::evalg() {
    if(ds->getUpdateg()) ds->updateg();
    return g;
  }

  const Vec& Link::evalgd() {
    if(ds->getUpdategd()) ds->updategd();
    return gd;
  }

  const Vec& Link::evalla() {
    if(ds->getUpdatela()) ds->updatela();
    return la;
  }

  const Vec& Link::evalLa() {
    if(ds->getUpdateLa()) ds->updateLa();
    return La;
  }

  const Vec& Link::evalwb() {
    if(ds->getUpdatewb()) ds->updatewb();
    return wb;
  }

  const Vec& Link::evalxd() {
    if(ds->getUpdatezd()) ds->updatezd();
    return xd;
  }

  const fmatvec::Vec& Link::getla(bool check) const {
    assert((not check) or (not ds->getUpdatela()));
    return la;
  }

  fmatvec::Vec& Link::getla(bool check) {
    assert((not check) or (not ds->getUpdatela()));
    return la;
  }

  const fmatvec::Vec& Link::getLa(bool check) const {
    assert((not check) or (not ds->getUpdateLa()));
    return La;
  }

  fmatvec::Vec& Link::getLa(bool check) {
    assert((not check) or (not ds->getUpdateLa()));
    return La;
  }

  fmatvec::Vec& Link::getwb(bool check) {
    assert((not check) or (not ds->getUpdatewb()));
    return wb;
  }

  void Link::createPlotGroup() {
    plotGroup=parent->getLinksPlotGroup()->createChildObject<H5::Group>(name)();
    plotGroup->createChildAttribute<H5::SimpleAttribute<string>>("Description")()->write("Object of class: "+boost::core::demangle(typeid(*this).name()));
    plotColumns.insert(plotColumns.begin(), "time");
  }

}
