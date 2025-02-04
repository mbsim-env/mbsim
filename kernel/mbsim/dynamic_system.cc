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
#include "mbsim/dynamic_system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/contours/rigid_contour.h"
#include "mbsim/modelling_interface.h"
#include "mbsim/links/link.h"
#include "mbsim/objects/object.h"
#include "mbsim/constraints/constraint.h"
#include "mbsim/observers/observer.h"
#include "mbsim/dynamic_system_solver.h"
#include "hdf5serie/file.h"
#include "hdf5serie/simpleattribute.h"

#include "openmbvcppinterface/group.h"
#include <openmbvcppinterface/frame.h>

//#ifdef _OPENMP
//#include <omp.h>
//#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;

namespace MBSim {

  DynamicSystem::DynamicSystem(const string &name) :
      Element(name), R(nullptr), qSize(0), qInd(0), xSize(0), xInd(0), isSize(0), isInd(0), gSize(0), gInd(0), gdSize(0), gdInd(0), laSize(0), laInd(0), rFactorSize(0), rFactorInd(0), svSize(0), svInd(0), LinkStatusSize(0), LinkStatusInd(0), LinkStatusRegSize(0), LinkStatusRegInd(0), corrInd(0)
  {
    uSize[0] = 0;
    uSize[1] = 0;
    uInd[0] = 0;
    uInd[1] = 0;
    hSize[0] = 0;
    hSize[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;

    I = new Frame("I");
    addFrame(I);
  }

  DynamicSystem::~DynamicSystem() {
    for (auto & i : dynamicsystem)
      delete i;
    for (auto & i : object)
      delete i;
    for (auto & i : link)
      delete i;
    for (auto & i : constraint)
      delete i;
    for (auto & i : frame)
      delete i;
    for (auto & i : contour)
      delete i;
    for (auto & i : observer)
      delete i;
    for (auto & i : inverseKineticsLink)
      delete i;
  }

  void DynamicSystem::updateT() {
    for (auto & i : dynamicsystem)
      (*i).updateT();

    for (auto & i : object)
      (*i).updateT();
  }

  void DynamicSystem::updateM() {
    for (auto & i : dynamicsystem)
      i->updateM();

    for (auto & i : objectWithNonConstantMassMatrix) {
      i->getM(false).init(0);
      i->updateM();
    }
  }

  void DynamicSystem::updateLLM() {
    for(auto & i : dynamicsystem)
      i->updateLLM();

    for(auto & i : objectWithNonConstantMassMatrix)
      i->updateLLM();
  }

  void DynamicSystem::updateh(int k) {
    for (auto & i : dynamicsystem)
      i->updateh(k);

    for (auto & i : object)
      i->updateh(k);

    for(auto & i : linkSingleValued)
	i->updateh(k);
  }

  void DynamicSystem::updatedq() {
    for (auto & i : dynamicsystem)
      (*i).updatedq();

    for (auto & i : object)
      (*i).updatedq();
  }

  void DynamicSystem::updatedu() {
    for(auto & i : dynamicsystem)
      i->updatedu();

    for(auto & i : object)
      i->updatedu();
  }

  void DynamicSystem::updatedx() {
    for (auto & i : link)
      (*i).updatedx();

    for (auto & i : constraint)
      (*i).updatedx();
  }

  void DynamicSystem::updateqd() {
    for (auto & i : dynamicsystem)
      (*i).updateqd();

    for (auto & i : object)
      (*i).updateqd();
  }

  void DynamicSystem::updateud() {
    for(auto & i : dynamicsystem)
      i->updateud();

    for(auto & i : object)
      i->updateud();
  }

  void DynamicSystem::updatexd() {
    for (auto & i : link)
      (*i).updatexd();

    for (auto & i : constraint)
      (*i).updatexd();
  }

  void DynamicSystem::updatezd() {
    for(auto & i : dynamicsystem)
      i->updatezd();

    for(auto & i : object) {
      i->updateqd();
      i->updateud();
    }

    for(auto & i : link)
      i->updatexd();

    for(auto & i : constraint)
      i->updatexd();
  }

  void DynamicSystem::updatewb() {

    for (auto & i : linkSetValuedActive)
      (*i).updatewb();
  }

  void DynamicSystem::updateW(int j) {

    for (auto & i : linkSetValuedActive)
      (*i).updateW(j);
  }

  void DynamicSystem::updateV(int j) {

    for (auto & i : linkSetValuedActive)
      (*i).updateV(j);
  }

  void DynamicSystem::updater(int j) {

    for (auto & i : linkSetValuedActive)
      (*i).updater(j);
  }

  void DynamicSystem::updateJrla(int j) {

    for (auto & i : linkSetValuedActive)
      (*i).updateJrla(j);
  }

  void DynamicSystem::updateg() {

    for (auto & i : linkSetValuedActive)
      i->updateg();
  }

  void DynamicSystem::updategd() {

    for (auto & i : linkSetValuedActive)
      i->updategd();
  }

  void DynamicSystem::updateStopVector() {
    for (auto & i : linkWithStopVector)
      i->updateStopVector();
  }

  void DynamicSystem::updateStopVectorParameters() {
    for (auto & i : linkWithStopVector)
      i->updateStopVectorParameters();
  }

  void DynamicSystem::updateLinkStatus() {
    for (auto & i : linkSetValued)
      i->updateLinkStatus();
  }

  void DynamicSystem::updateLinkStatusReg() {
    for (auto & i : linkSingleValued)
      i->updateLinkStatusReg();
  }

  void DynamicSystem::updateWInverseKinetics() {
    WInverseKinetics.init(0);

    for (auto & i : inverseKineticsLink)
      (*i).updateW(1);
  }

  void DynamicSystem::updatebInverseKinetics() {
    bInverseKinetics.init(0);

    for (auto & i : inverseKineticsLink)
      (*i).updateb();
  }

  void DynamicSystem::sethSize(int hSize_, int j) {
    hSize[j] = hSize_;

    for (auto & i : dynamicsystem)
      i->sethSize(i->getuSize(j), j);

    for (auto & i : object)
      i->sethSize(i->getuSize(j), j);
  }

  void DynamicSystem::sethInd(int hInd_, int j) {
    hInd[j] = hInd_;

    for (auto & i : dynamicsystem)
      i->sethInd(i->getuInd(j), j);

    for (auto & i : object)
      i->sethInd(i->getuInd(j), j);
  }

  void DynamicSystem::calcqSize() {
    qSize = 0;

    for (auto & i : dynamicsystem) {
      i->calcqSize();
      qSize += i->getqSize();
    }
    for (auto & i : object) {
      i->calcqSize();
      qSize += i->getqSize();
    }
  }

  void DynamicSystem::setsvInd(int svInd_) {
    svInd = svInd_;

    for (auto & i : dynamicsystem) {
      i->setsvInd(svInd_);
      svInd_ += i->getsvSize();
    }
    for (auto & i : link) {
      i->setsvInd(svInd_);
      svInd_ += i->getsvSize();
    }
  }

  void DynamicSystem::setqInd(int qInd_) {
    qInd = qInd_;

    for (auto & i : dynamicsystem) {
      i->setqInd(qInd_);
      qInd_ += i->getqSize();
    }
    for (auto & i : object) {
      i->setqInd(qInd_);
      qInd_ += i->getqSize();
    }
  }

  void DynamicSystem::calcuSize(int j) {
    uSize[j] = 0;

    for (auto & i : dynamicsystem) {
      i->calcuSize(j);
      uSize[j] += i->getuSize(j);
    }
    for (auto & i : object) {
      i->calcuSize(j);
      uSize[j] += i->getuSize(j);
    }
  }

  void DynamicSystem::setuInd(int uInd_, int j) {
    uInd[j] = uInd_;

    for (auto & i : dynamicsystem) {
      i->setuInd(uInd_, j);
      uInd_ += i->getuSize(j);
    }
    for (auto & i : object) {
      i->setuInd(uInd_, j);
      uInd_ += i->getuSize(j);
    }
  }

  void DynamicSystem::setDynamicSystemSolver(DynamicSystemSolver* sys) {
    Element::setDynamicSystemSolver(sys);
    for (auto & i : dynamicsystem)
      i->setDynamicSystemSolver(sys);

    for (auto & i : object)
      i->setDynamicSystemSolver(sys);

    for (auto & i : link)
      i->setDynamicSystemSolver(sys);

    for (auto & i : constraint)
      i->setDynamicSystemSolver(sys);

    for (auto & i : frame)
      i->setDynamicSystemSolver(sys);

    for (auto & i : contour)
      i->setDynamicSystemSolver(sys);

    for (auto & i : inverseKineticsLink)
      i->setDynamicSystemSolver(sys);

    for (auto & i : observer)
      i->setDynamicSystemSolver(sys);
  }

  void DynamicSystem::plot() {
    for (auto & i : dynamicsystem)
      i->plot();
    for (auto & i : object)
      i->plot();
    for (auto & i : link)
      i->plot();
    for (auto & i : constraint)
      i->plot();
    for (auto & i : frame)
      i->plot();
    for (auto & i : contour)
      i->plot();
    for (auto & i : inverseKineticsLink)
      i->plot();
    for (auto & i : observer)
      i->plot();
  }
  
  void DynamicSystem::plotAtSpecialEvent() {
    for (auto & i : dynamicsystem)
      i->plotAtSpecialEvent();
    for (auto & i : object)
      i->plotAtSpecialEvent();
    for (auto & i : link)
      i->plotAtSpecialEvent();
    for (auto & i : constraint)
      i->plotAtSpecialEvent();
    for (auto & i : frame)
      i->plotAtSpecialEvent();
    for (auto & i : contour)
      i->plotAtSpecialEvent();
    for (auto & i : inverseKineticsLink)
      i->plotAtSpecialEvent();
    for (auto & i : observer)
      i->plotAtSpecialEvent();
  }

  void DynamicSystem::init(InitStage stage, const InitConfigSet &config) {
    if (stage ==resolveStringRef) {
      if (!saved_frameOfReference.empty())
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
    }
    else if (stage == preInit) {
      if (parent)
        updatePlotFeatures();
      if (!R) {
        auto *sys = dynamic_cast<DynamicSystem*>(parent);
        if (sys)
          R = sys->getFrameI();
      }
      for (unsigned int k = 1; k < frame.size(); k++) {
        if(not(static_cast<FixedRelativeFrame*>(frame[k])->getFrameOfReference()))
          static_cast<FixedRelativeFrame*>(frame[k])->setFrameOfReference(I);
      }
      for (auto & k : contour) {
        if(not(static_cast<RigidContour*>(k))->getFrameOfReference())
          static_cast<RigidContour*>(k)->setFrameOfReference(I);
      }
      if (R) {
        I->setOrientation(R->evalOrientation());
        I->setPosition(R->getPosition());
      }
    }
    else if (stage == plotting) {
      if (plotFeature[plotRecursive]) {
        if (!parent)
          plotGroup = hdf5File.get();
        else
          plotGroup = parent->getGroupsPlotGroup()->createChildObject<H5::Group>(name)();

        framesPlotGroup = plotGroup->createChildObject<H5::Group>("frames")();
        contoursPlotGroup = plotGroup->createChildObject<H5::Group>("contours")();
        groupsPlotGroup = plotGroup->createChildObject<H5::Group>("groups")();
        objectsPlotGroup = plotGroup->createChildObject<H5::Group>("objects")();
        linksPlotGroup = plotGroup->createChildObject<H5::Group>("links")();
        constraintsPlotGroup = plotGroup->createChildObject<H5::Group>("constraints")();
        observersPlotGroup = plotGroup->createChildObject<H5::Group>("observers")();

        plotGroup->createChildAttribute<H5::SimpleAttribute<string>>("Description")()->write(string("Object of class: ")+boost::core::demangle(typeid(*this).name()));
        plotVectorSerie = nullptr;
      }
      if (plotFeature[openMBV]) {
        openMBVGrp = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
        openMBVGrp->setName(name);
        if (parent)
          parent->getGroupsOpenMBVGrp()->addObject(openMBVGrp);
	framesOpenMBVGrp = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
	framesOpenMBVGrp->setName("frames");
	openMBVGrp->addObject(framesOpenMBVGrp);
	contoursOpenMBVGrp = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
	contoursOpenMBVGrp->setName("contours");
	openMBVGrp->addObject(contoursOpenMBVGrp);
	groupsOpenMBVGrp = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
	groupsOpenMBVGrp->setName("groups");
	openMBVGrp->addObject(groupsOpenMBVGrp);
	objectsOpenMBVGrp = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
	objectsOpenMBVGrp->setName("objects");
	openMBVGrp->addObject(objectsOpenMBVGrp);
	linksOpenMBVGrp = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
	linksOpenMBVGrp->setName("links");
	openMBVGrp->addObject(linksOpenMBVGrp);
	constraintsOpenMBVGrp = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
	constraintsOpenMBVGrp->setName("constraints");
	openMBVGrp->addObject(constraintsOpenMBVGrp);
	observersOpenMBVGrp = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
	observersOpenMBVGrp->setName("observers");
	openMBVGrp->addObject(observersOpenMBVGrp);
      }
      Element::init(stage, config);
    }

    for (auto & i : frame)
      i->init(stage, config);
    for (auto & i : contour)
      i->init(stage, config);
    for (auto & i : dynamicsystem)
      i->init(stage, config);
    for (auto & i : object)
      i->init(stage, config);
    for (auto & i : link)
      i->init(stage, config);
    for (auto & i : constraint)
      i->init(stage, config);
    for (auto & i : model)
      i->init(stage, config);
    for (auto & i : inverseKineticsLink)
      i->init(stage, config);
    for (auto & i : observer)
      i->init(stage, config);
  }

  int DynamicSystem::solveConstraintsFixpointSingle() {

    for (auto & i : linkSetValuedActive)
      i->solveConstraintsFixpointSingle();

    return 0;
  }

  int DynamicSystem::solveImpactsFixpointSingle() {

    for (auto & i : linkSetValuedActive)
      i->solveImpactsFixpointSingle();

    return 0;
  }

  int DynamicSystem::solveConstraintsGaussSeidel() {

    for (auto & i : linkSetValuedActive)
      i->solveConstraintsGaussSeidel();

    return 0;
  }

  int DynamicSystem::solveImpactsGaussSeidel() {

    for (auto & i : linkSetValuedActive)
      i->solveImpactsGaussSeidel();

    return 0;
  }

  int DynamicSystem::solveConstraintsRootFinding() {

    for (auto & i : linkSetValuedActive)
      i->solveConstraintsRootFinding();

    return 0;
  }

  int DynamicSystem::solveImpactsRootFinding() {

    for (auto & i : linkSetValuedActive)
      i->solveImpactsRootFinding();

    return 0;
  }

  int DynamicSystem::jacobianConstraints() {

    for (auto & i : linkSetValuedActive)
      i->jacobianConstraints();

    return 0;
  }

  int DynamicSystem::jacobianImpacts() {

    for (auto & i : linkSetValuedActive)
      i->jacobianImpacts();

    return 0;
  }

  void DynamicSystem::checkConstraintsForTermination() {

    for (auto & i : linkSetValuedActive)
      (*i).checkConstraintsForTermination();
  }

  void DynamicSystem::checkImpactsForTermination() {

    for (auto & i : linkSetValuedActive)
      (*i).checkImpactsForTermination();
  }

  void DynamicSystem::updaterFactors() {

    for (auto & i : linkSetValuedActive)
      (*i).updaterFactors();
  }

  Frame* DynamicSystem::getFrame(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < frame.size(); i++) {
      if (frame[i]->getName() == name)
        return frame[i];
    }
    if (check) {
      if (!(i < frame.size()))
        throwError("DynamicSystem comprises no frame \"" + name + "\"!");
      assert(i < frame.size());
    }
    return nullptr;
  }

  Contour* DynamicSystem::getContour(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < contour.size(); i++) {
      if (contour[i]->getName() == name)
        return contour[i];
    }
    if (check) {
      if (!(i < contour.size()))
        throwError("DynamicSystem comprises no contour \"" + name + "\"!");
      assert(i < contour.size());
    }
    return nullptr;
  }

  void DynamicSystem::updateqRef(Vec &qParent) {
    q.ref(qParent, RangeV(qInd, qInd + qSize - 1));

    for (auto & i : dynamicsystem)
      (*i).updateqRef(qParent);

    for (auto & i : object)
      (*i).updateqRef(qParent);
  }

  void DynamicSystem::updateqdRef(Vec &qdParent) {
    qd.ref(qdParent, RangeV(qInd, qInd + qSize - 1));

    for (auto & i : dynamicsystem)
      (*i).updateqdRef(qdParent);

    for (auto & i : object)
      (*i).updateqdRef(qdParent);
  }

  void DynamicSystem::updatedqRef(Vec &dqParent) {
    dq.ref(dqParent, RangeV(qInd, qInd + qSize - 1));

    for (auto & i : dynamicsystem)
      (*i).updatedqRef(dqParent);

    for (auto & i : object)
      (*i).updatedqRef(dqParent);
  }

  void DynamicSystem::updateuRef(Vec &uParent) {
    u.ref(uParent, RangeV(uInd[0], uInd[0] + uSize[0] - 1));

    for (auto & i : dynamicsystem)
      (*i).updateuRef(uParent);

    for (auto & i : object)
      (*i).updateuRef(uParent);
  }

  void DynamicSystem::updateuallRef(Vec &uParent) {
    for (auto & i : dynamicsystem)
      (*i).updateuallRef(uParent);

    for (auto & i : object)
      (*i).updateuallRef(uParent);
  }

  void DynamicSystem::updateudRef(Vec &udParent) {
    ud.ref(udParent, RangeV(uInd[0], uInd[0] + uSize[0] - 1));

    for (auto & i : dynamicsystem)
      (*i).updateudRef(udParent);

    for (auto & i : object)
      (*i).updateudRef(udParent);
  }

  void DynamicSystem::updateduRef(Vec &duParent) {
    du.ref(duParent, RangeV(uInd[0], uInd[0] + uSize[0] - 1));

    for (auto & i : dynamicsystem)
      (*i).updateduRef(duParent);

    for (auto & i : object)
      (*i).updateduRef(duParent);
  }

  void DynamicSystem::updateudallRef(Vec &udParent) {
    for (auto & i : dynamicsystem)
      (*i).updateudallRef(udParent);

    for (auto & i : object)
      (*i).updateudallRef(udParent);
  }

  void DynamicSystem::updatexRef(Vec &xParent) {
    x.ref(xParent, RangeV(xInd, xInd + xSize - 1));

    for (auto & i : dynamicsystem)
      (*i).updatexRef(xParent);

    for (auto & i : link)
      (*i).updatexRef(xParent);

    for (auto & i : constraint)
      (*i).updatexRef(xParent);
  }

  void DynamicSystem::updatexdRef(Vec &xdParent) {
    xd.ref(xdParent, RangeV(xInd, xInd + xSize - 1));

    for (auto & i : dynamicsystem)
      (*i).updatexdRef(xdParent);

    for (auto & i : link)
      (*i).updatexdRef(xdParent);

    for (auto & i : constraint)
      (*i).updatexdRef(xdParent);
  }

  void DynamicSystem::updatedxRef(Vec &dxParent) {
    dx.ref(dxParent, RangeV(xInd, xInd + xSize - 1));

    for (auto & i : dynamicsystem)
      (*i).updatedxRef(dxParent);

    for (auto & i : link)
      (*i).updatedxRef(dxParent);

    for (auto & i : constraint)
      (*i).updatedxRef(dxParent);
  }

  void DynamicSystem::updatehRef(Vec &hParent, int j) {
    h[j].ref(hParent, RangeV(hInd[j], hInd[j] + hSize[j] - 1));

    for (auto & i : dynamicsystem)
      (*i).updatehRef(hParent, j);

    for (auto & i : object)
      (*i).updatehRef(hParent, j);

    for (auto & i : linkSingleValued)
      (*i).updatehRef(hParent, j);
  }

  void DynamicSystem::updaterRef(Vec &rParent, int j) {
    r[j].ref(rParent, RangeV(hInd[j], hInd[j] + hSize[j] - 1));

    for (auto & i : dynamicsystem)
      (*i).updaterRef(rParent, j);

    for (auto & i : object)
      (*i).updaterRef(rParent, j);

    for (auto & i : linkSetValuedActive)
      (*i).updaterRef(rParent, j);
  }

  void DynamicSystem::updateJrlaRef(Mat &JrlaParent, int j) {
    Jrla[j].ref(JrlaParent, RangeV(hInd[j], hInd[j] + hSize[j] - 1), RangeV(laInd, laInd + laSize - 1));

    for (auto & i : dynamicsystem)
      (*i).updateJrlaRef(JrlaParent, j);

    for (auto & i : linkSetValuedActive)
      (*i).updateJrlaRef(JrlaParent, j);
  }

  void DynamicSystem::updaterdtRef(Vec &rdtParent) {
    rdt.ref(rdtParent, RangeV(hInd[0], hInd[0] + hSize[0] - 1));

    for (auto & i : dynamicsystem)
      (*i).updaterdtRef(rdtParent);

    for (auto & i : object)
      (*i).updaterdtRef(rdtParent);
  }

  void DynamicSystem::updateTRef(Mat& TParent) {
    T.ref(TParent, RangeV(qInd, qInd + qSize - 1), RangeV(uInd[0], uInd[0] + uSize[0] - 1));

    for (auto & i : dynamicsystem)
      i->updateTRef(TParent);

    for (auto & i : object)
      i->updateTRef(TParent);
  }

  void DynamicSystem::updateMRef(SymMat& MParent) {
    M.ref(MParent, RangeV(hInd[0], hInd[0] + hSize[0] - 1));

    for (auto & i : dynamicsystem)
      i->updateMRef(MParent);

    for (auto & i : object)
      i->updateMRef(MParent);
  }

  void DynamicSystem::updateLLMRef(SymMat& LLMParent) {
    LLM.ref(LLMParent, RangeV(hInd[0], hInd[0] + hSize[0] - 1));

    for (auto & i : dynamicsystem)
      i->updateLLMRef(LLMParent);

    for (auto & i : object)
      i->updateLLMRef(LLMParent);
  }

  void DynamicSystem::updateInternalStateRef(Vec &curisParent, Vec &nextisParent) {
    curis.ref(curisParent, RangeV(isInd, isInd + isSize - 1));
    nextis.ref(nextisParent, RangeV(isInd, isInd + isSize - 1));

    for(auto & ds : dynamicsystem)
      ds->updateInternalStateRef(curisParent, nextisParent);

    for(auto & l : link)
      l->updateInternalStateRef(curisParent, nextisParent);

    for(auto & c : constraint)
      c->updateInternalStateRef(curisParent, nextisParent);

    for(auto & o : observer)
      o->updateInternalStateRef(curisParent, nextisParent);
  }

  void DynamicSystem::updategRef(Vec& gParent) {
    g.ref(gParent, RangeV(gInd, gInd + gSize - 1));

    for (auto & i : linkSetValued)
      (*i).updategRef(gParent);
  }

  void DynamicSystem::updategdRef(Vec& gdParent) {
    gd.ref(gdParent, RangeV(gdInd, gdInd + gdSize - 1));

    for (auto & i : linkSetValued)
      (*i).updategdRef(gdParent);
  }

  void DynamicSystem::updatelaRef(Vec &laParent) {
    la.ref(laParent, RangeV(laInd, laInd + laSize - 1));

    for (auto & i : linkSetValued)
      (*i).updatelaRef(laParent);
  }

  void DynamicSystem::updateLaRef(Vec &LaParent) {
    La.ref(LaParent, RangeV(laInd, laInd + laSize - 1));

    for (auto & i : linkSetValued)
      (*i).updateLaRef(LaParent);
  }

  void DynamicSystem::updatelaInverseKineticsRef(Vec &laParent) {
    laInverseKinetics.ref(laParent, RangeV(0, laInverseKineticsSize - 1));

    for (auto & i : inverseKineticsLink)
      (*i).updatelaRef(laParent);
  }

  void DynamicSystem::updatewbRef(Vec &wbParent) {
    wb.ref(wbParent, RangeV(laInd, laInd + laSize - 1));

    for (auto & i : linkSetValued)
      (*i).updatewbRef(wbParent);
  }

  void DynamicSystem::updateWRef(Mat &WParent, int j) {
    W[j].ref(WParent, RangeV(hInd[j], hInd[j] + hSize[j] - 1), RangeV(laInd, laInd + laSize - 1));

    for (auto & i : linkSetValued)
      (*i).updateWRef(WParent, j);
  }

  void DynamicSystem::updateWInverseKineticsRef(Mat &WParent) {
    WInverseKinetics.ref(WParent, RangeV(hInd[1], hInd[1] + hSize[1] - 1), RangeV(0, laInverseKineticsSize - 1));

    for (auto & i : inverseKineticsLink)
      (*i).updateWRef(WParent, 1);
  }

  void DynamicSystem::updatebInverseKineticsRef(Mat &bParent) {
    bInverseKinetics.ref(bParent, RangeV(0, bInverseKineticsSize - 1), RangeV(0, laInverseKineticsSize - 1));

    for (auto & i : inverseKineticsLink)
      (*i).updatebRef(bParent);
  }

  void DynamicSystem::updateVRef(Mat &VParent, int j) {
    V[j].ref(VParent, RangeV(hInd[j], hInd[j] + hSize[j] - 1), RangeV(laInd, laInd + laSize - 1));

    for (auto & i : linkSetValued)
      (*i).updateVRef(VParent, j);
  }

  void DynamicSystem::updatesvRef(Vec &svParent) {
    sv.ref(svParent, RangeV(svInd, svInd + svSize - 1));

    for (auto & i : dynamicsystem)
      i->updatesvRef(svParent);

    for (auto & i : link)
      (*i).updatesvRef(svParent);
  }

  void DynamicSystem::updatejsvRef(VecInt &jsvParent) {
    jsv.ref(jsvParent, RangeV(svInd, svInd + svSize - 1));

    for (auto & i : dynamicsystem)
      i->updatejsvRef(jsvParent);

    for (auto & i : link)
      (*i).updatejsvRef(jsvParent);
  }

  void DynamicSystem::updateresRef(Vec &resParent) {
    res.ref(resParent, RangeV(laInd, laInd + laSize - 1));

    for (auto & i : linkSetValued)
      (*i).updateresRef(resParent);
  }

  void DynamicSystem::updaterFactorRef(Vec &rFactorParent) {
    rFactor.ref(rFactorParent, RangeV(rFactorInd, rFactorInd + rFactorSize - 1));

    for (auto & i : linkSetValued)
      (*i).updaterFactorRef(rFactorParent);
  }

  void DynamicSystem::updateLinkStatusRef(VecInt &LinkStatusParent) {
    LinkStatus.ref(LinkStatusParent, RangeV(LinkStatusInd, LinkStatusInd + LinkStatusSize - 1));

    for (auto & i : dynamicsystem)
      i->updateLinkStatusRef(LinkStatusParent);

    for (auto & i : linkSetValued)
      (*i).updateLinkStatusRef(LinkStatusParent);
  }

  void DynamicSystem::updateLinkStatusRegRef(VecInt &LinkStatusRegParent) {
    LinkStatusReg.ref(LinkStatusRegParent, RangeV(LinkStatusRegInd, LinkStatusRegInd + LinkStatusRegSize - 1));

    for (auto & i : dynamicsystem)
      i->updateLinkStatusRegRef(LinkStatusRegParent);

    for (auto & i : linkSingleValued)
      (*i).updateLinkStatusRegRef(LinkStatusRegParent);
  }

  void DynamicSystem::initz() {
    for (auto & i : dynamicsystem)
      i->initz();
    for (auto & i : object)
      i->initz();
    for (auto & i : link)
      i->initz();
    for (auto & i : constraint)
      i->initz();
  }

  void DynamicSystem::writez(H5::GroupBase *parent) {
    for (unsigned i = 0; i < dynamicsystem.size(); i++) {
      H5::Group *group = parent->createChildObject<H5::Group>("System_" + to_string(i))();
      dynamicsystem[i]->writez(group);
    }
    for (unsigned i = 0; i < object.size(); i++) {
      H5::Group *group = parent->createChildObject<H5::Group>("Object_" + to_string(i))();
      object[i]->writez(group);
    }
    for (unsigned i = 0; i < link.size(); i++) {
      H5::Group *group = parent->createChildObject<H5::Group>("Link_" + to_string(i))();
      link[i]->writez(group);
    }
  }

  void DynamicSystem::readz0(H5::GroupBase *parent) {
    for (unsigned i = 0; i < dynamicsystem.size(); i++) {
      auto *group = parent->openChildObject<H5::Group>("System_" + to_string(i));
      dynamicsystem[i]->readz0(group);
    }
    for (unsigned i = 0; i < object.size(); i++) {
      auto *group = parent->openChildObject<H5::Group>("Object_" + to_string(i));
      object[i]->readz0(group);
    }
    for (unsigned i = 0; i < link.size(); i++) {
      auto *group = parent->openChildObject<H5::Group>("Link_" + to_string(i));
      link[i]->readz0(group);
    }
  }

  void DynamicSystem::clearElementLists() {
    for (auto & i : dynamicsystem)
      i->clearElementLists();
    dynamicsystem.clear(); // delete old DynamicSystem list
    object.clear(); // delete old object list
    frame.clear(); // delete old frame list
    contour.clear(); // delete old contour list
    link.clear(); // delete old link list
    constraint.clear(); // delete old constraint list
    inverseKineticsLink.clear(); // delete old link list
    observer.clear(); // delete old link list
  }

  void DynamicSystem::buildListOfDynamicSystems(vector<DynamicSystem*> &sys) {
    for (auto & i : dynamicsystem) {
      sys.push_back(i);
      i->setPath(i->getPath());
    }
    for (auto & i : dynamicsystem)
      i->buildListOfDynamicSystems(sys);
  }

  void DynamicSystem::buildListOfObjects(vector<Object*> &obj) {
    for (auto & i : object) {
      obj.push_back(i);
      i->setPath(i->getPath());
    }
    for (auto & i : dynamicsystem)
      i->buildListOfObjects(obj);
  }

  void DynamicSystem::buildListOfLinks(vector<Link*> &lnk) {
    for (auto & i : link) {
      lnk.push_back(i);
      i->setPath(i->getPath());
    }
    for (auto & i : dynamicsystem)
      i->buildListOfLinks(lnk);
  }

  void DynamicSystem::buildListOfConstraints(vector<Constraint*> &crt) {
    for (auto & i : constraint) {
      crt.push_back(i);
      i->setPath(i->getPath());
    }
    for (auto & i : dynamicsystem)
      i->buildListOfConstraints(crt);
  }

  void DynamicSystem::buildListOfFrames(vector<Frame*> &frm) {
    for (auto & i : frame) {
      frm.push_back(i);
      i->setPath(i->getPath());
    }
    for (auto & i : dynamicsystem)
      i->buildListOfFrames(frm);
  }

  void DynamicSystem::buildListOfContours(vector<Contour*> &cnt) {
    for (auto & i : contour) {
      cnt.push_back(i);
      i->setPath(i->getPath());
    }
    for (auto & i : dynamicsystem)
      i->buildListOfContours(cnt);
  }

  void DynamicSystem::buildListOfModels(std::vector<ModellingInterface*> &modelList) {
    for (auto i : model) {
      modelList.push_back(i);
      // Note! setPath is (and must be) done in processModellList
    }
    for (auto & i : dynamicsystem)
      i->buildListOfModels(modelList);
  }

  void DynamicSystem::buildListOfInverseKineticsLinks(vector<Link*> &iklnk) {
    for (auto & i : inverseKineticsLink) {
      iklnk.push_back(i);
      i->setPath(i->getPath());
    }
    for (auto & i : dynamicsystem)
      i->buildListOfInverseKineticsLinks(iklnk);
  }

  void DynamicSystem::buildListOfObservers(vector<Observer*> &obsrv) {
    for (auto & i : observer) {
      obsrv.push_back(i);
      i->setPath(i->getPath());
    }
    for (auto & i : dynamicsystem)
      i->buildListOfObservers(obsrv);
  }

  void DynamicSystem::setUpInverseKinetics() {
    for (auto & i : dynamicsystem)
      i->setUpInverseKinetics();

    for (auto & i : object)
      i->setUpInverseKinetics();

    for (auto & i : constraint)
      i->setUpInverseKinetics();
  }

  void DynamicSystem::setUpLinks() {
    for (auto & i : dynamicsystem)
      i->setUpLinks();

    // clear container first, because setUpLinks in called twice from InitStage resize (before and after the reorganization)
    linkSetValued.clear();
    linkSetValuedActive.clear();
    linkSingleValued.clear();
    linkWithStopVector.clear();
    for (auto & i : link) {
      if (i->isSetValued()) {
        linkSetValued.push_back(i);
        linkSetValuedActive.push_back(i);
      }
      if (i->isSingleValued())
        linkSingleValued.push_back(i);
      if (i->getsvSize()>0)
        linkWithStopVector.push_back(i);
    }
  }

  bool DynamicSystem::gActiveChanged() {
    bool changed = false;

    for (auto & i : linkSetValued)
      if (i->gActiveChanged())
        changed = true;

    return changed;
  }

  bool DynamicSystem::gActiveChangedReg() {
    bool changed = false;

    for (auto & i : linkSingleValued)
      if (i->gActiveChanged())
        changed = true;
    
    return changed;
  }

  bool DynamicSystem::detectImpact() {
    bool impact = false;

    for (auto & i : linkSetValued)
      if (i->detectImpact())
        impact = true;

    return impact;
  }

  void DynamicSystem::calcxSize() {
    xSize = 0;

    for (auto & i : dynamicsystem) {
      i->calcxSize();
      xSize += i->getxSize();
    }

    for (auto & i : link) {
      i->calcxSize();
      xSize += i->getxSize();
    }

    for (auto & i : constraint) {
      i->calcxSize();
      xSize += i->getxSize();
    }
  }

  void DynamicSystem::setxInd(int xInd_) {
    xInd = xInd_;

    for (auto & i : dynamicsystem) {
      i->setxInd(xInd_);
      xInd_ += i->getxSize();
    }

    for (auto & i : link) {
      i->setxInd(xInd_);
      xInd_ += i->getxSize();
    }

    for (auto & i : constraint) {
      i->setxInd(xInd_);
      xInd_ += i->getxSize();
    }
  }

  void DynamicSystem::calcLinkStatusSize() {
    LinkStatusSize = 0;

    for (auto & i : dynamicsystem) {
      i->calcLinkStatusSize();
      i->setLinkStatusInd(LinkStatusSize);
      LinkStatusSize += i->getLinkStatusSize();
    }
    for (auto & i : linkSetValued) {
      i->calcLinkStatusSize();
      i->setLinkStatusInd(LinkStatusSize);
      LinkStatusSize += i->getLinkStatusSize();
    }
  }

  void DynamicSystem::calcLinkStatusRegSize() {
    LinkStatusRegSize = 0;

    for (auto & i : dynamicsystem) {
      i->calcLinkStatusRegSize();
      i->setLinkStatusRegInd(LinkStatusRegSize);
      LinkStatusRegSize += i->getLinkStatusRegSize();
    }
    for (auto & i : linkSingleValued) {
      i->calcLinkStatusRegSize();
      i->setLinkStatusRegInd(LinkStatusRegSize);
      LinkStatusRegSize += i->getLinkStatusRegSize();
    }
  }

  void DynamicSystem::calcsvSize() {
    svSize = 0;

    for (auto & i : dynamicsystem) {
      i->calcsvSize();
      svSize += i->getsvSize();
    }
    for (auto & i : link) {
      i->calcsvSize();
      svSize += i->getsvSize();
    }
  }

  void DynamicSystem::calclaSize(int j) {
    laSize = 0;

    for (auto & i : linkSetValued) {
      i->calclaSize(j);
      i->setlaInd(laSize);
      laSize += i->getlaSize();
    }
  }

  void DynamicSystem::calclaInverseKineticsSize() {
    laInverseKineticsSize = 0;

    for (auto & i : inverseKineticsLink) {
      i->calclaSize(0);
      i->setlaInd(laInverseKineticsSize);
      laInverseKineticsSize += i->getlaSize();
    }
  }

  void DynamicSystem::calcbInverseKineticsSize() {
    bInverseKineticsSize = 0;

    for (auto & i : inverseKineticsLink) {
      i->calcbSize();
      i->setbInd(bInverseKineticsSize);
      bInverseKineticsSize += i->getbSize();
    }
  }

  void DynamicSystem::calcisSize() {
    isSize = 0;

    for(auto & ds : dynamicsystem) {
      ds->calcisSize();
      ds->setisInd(isSize);
      isSize += ds->getisSize();
    }

    for(auto & l : link) {
      l->calcisSize();
      l->setisInd(isSize);
      isSize += l->getisSize();
    }

    for(auto & c : constraint) {
      c->calcisSize();
      c->setisInd(isSize);
      isSize += c->getisSize();
    }

    for(auto & o : observer) {
      o->calcisSize();
      o->setisInd(isSize);
      isSize += o->getisSize();
    }
  }

  void DynamicSystem::aboutToUpdateInternalState() {
    for(auto & ds : dynamicsystem)
      ds->aboutToUpdateInternalState();

    for(auto & l : link)
      l->aboutToUpdateInternalState();

    for(auto & c : constraint)
      c->aboutToUpdateInternalState();

    for(auto & o : observer)
      o->aboutToUpdateInternalState();
  }

  void DynamicSystem::postprocessing() {
    for(auto & ds : dynamicsystem)
      ds->postprocessing();

    for(auto & l : link)
      l->postprocessing();

    for(auto & c : constraint)
      c->postprocessing();

    for(auto & o : observer)
      o->postprocessing();
  }

  void DynamicSystem::calcgSize(int j) {
    gSize = 0;

    for (auto & i : linkSetValued) {
      i->calcgSize(j);
      i->setgInd(gSize);
      gSize += i->getgSize();
    }
  }

  void DynamicSystem::calcgdSize(int j) {
    gdSize = 0;

    for (auto & i : linkSetValued) {
      i->calcgdSize(j);
      i->setgdInd(gdSize);
      gdSize += i->getgdSize();
    }
  }

  void DynamicSystem::calcrFactorSize(int j) {
    rFactorSize = 0;

    for (auto & i : linkSetValued) {
      i->calcrFactorSize(j);
      i->setrFactorInd(rFactorSize);
      rFactorSize += i->getrFactorSize();
    }
  }

  void DynamicSystem::setUpObjectsWithNonConstantMassMatrix() {

    for (auto & i : object) {
      if (i->hasNonCostantMassMatrix())
        objectWithNonConstantMassMatrix.push_back(i);
    }
  }

  void DynamicSystem::setUpActiveLinks() {

    linkSetValuedActive.clear();

    for (auto & i : linkSetValued) {
      if (i->isActive())
        linkSetValuedActive.push_back(i);
    }
  }

  void DynamicSystem::checkActive(int j) {
    for (auto & i : linkWithStopVector)
      i->checkActive(j);
    setUpActiveLinks();
  }

  void DynamicSystem::setGeneralizedRelativePositionTolerance(double tol) {
    for (auto & i : dynamicsystem)
      (*i).setGeneralizedRelativePositionTolerance(tol);
    for (auto & i : link)
      (*i).setGeneralizedRelativePositionTolerance(tol);
  }

  void DynamicSystem::setGeneralizedRelativeVelocityTolerance(double tol) {
    for (auto & i : dynamicsystem)
      (*i).setGeneralizedRelativeVelocityTolerance(tol);
    for (auto & i : link)
      (*i).setGeneralizedRelativeVelocityTolerance(tol);
  }

  void DynamicSystem::setGeneralizedRelativeAccelerationTolerance(double tol) {
    for (auto & i : dynamicsystem)
      (*i).setGeneralizedRelativeAccelerationTolerance(tol);
    for (auto & i : link)
      (*i).setGeneralizedRelativeAccelerationTolerance(tol);
  }

  void DynamicSystem::setGeneralizedForceTolerance(double tol) {
    for (auto & i : dynamicsystem)
      (*i).setGeneralizedForceTolerance(tol);
    for (auto & i : link)
      (*i).setGeneralizedForceTolerance(tol);
  }

  void DynamicSystem::setGeneralizedImpulseTolerance(double tol) {
    for (auto & i : dynamicsystem)
      (*i).setGeneralizedImpulseTolerance(tol);
    for (auto & i : link)
      (*i).setGeneralizedImpulseTolerance(tol);
  }

  void DynamicSystem::setGeneralizedRelativePositionCorrectionValue(double corr) {
    for (auto & i : dynamicsystem)
      (*i).setGeneralizedRelativePositionCorrectionValue(corr);
    for (auto & i : link)
      (*i).setGeneralizedRelativePositionCorrectionValue(corr);
  }

  void DynamicSystem::setGeneralizedRelativeVelocityCorrectionValue(double corr) {
    for (auto & i : dynamicsystem)
      (*i).setGeneralizedRelativeVelocityCorrectionValue(corr);
    for (auto & i : link)
      (*i).setGeneralizedRelativeVelocityCorrectionValue(corr);
  }

  void DynamicSystem::setrMax(double rMax) {
    for (auto & i : dynamicsystem)
      (*i).setrMax(rMax);
    for (auto & i : link)
      (*i).setrMax(rMax);
  }

  void DynamicSystem::addFrame(FixedRelativeFrame *frame) {
    addFrame(static_cast<Frame*>(frame));
  }

  void DynamicSystem::addContour(RigidContour *contour) {
    addContour(static_cast<Contour*>(contour));
  }

  void DynamicSystem::addFrame(Frame *frame_) {
    if(frame_->getName().empty())
      throwError("A empty object name is not allowed!");
    if (getFrame(frame_->getName(), false)) {
      throwError("DynamicSystem can only comprise one Frame by the name \"" + frame_->getName() + "\"!");
      assert(getFrame(frame_->getName(),false)==nullptr);
    }
    frame.push_back(frame_);
    frame_->setParent(this);
  }

  void DynamicSystem::addContour(Contour* contour_) {
    if(contour_->getName().empty())
      throwError("A empty object name is not allowed!");
    if (getContour(contour_->getName(), false)) {
      throwError("DynamicSystem can only comprise one Contour by the name \"" + contour_->getName() + "\"!");
      assert(getContour(contour_->getName(),false)==nullptr);
    }
    contour.push_back(contour_);
    contour_->setParent(this);
  }

  int DynamicSystem::frameIndex(const Frame *frame_) const {
    for (unsigned int i = 0; i < frame.size(); i++) {
      if (frame_ == frame[i])
        return i;
    }
    return -1;
  }

  DynamicSystem* DynamicSystem::getGroup(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < dynamicsystem.size(); i++) {
      if (dynamicsystem[i]->getName() == name)
        return dynamicsystem[i];
    }
    if (check) {
      if (!(i < dynamicsystem.size()))
        throwError("DynamicSystem comprises no DynamicSystem \"" + name + "\"!");
      assert(i < dynamicsystem.size());
    }
    return nullptr;
  }

  Object* DynamicSystem::getObject(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < object.size(); i++) {
      if (object[i]->getName() == name)
        return object[i];
    }
    if (check) {
      if (!(i < object.size()))
        throwError("DynamicSystem comprises no Object \"" + name + "\"!");
      assert(i < object.size());
    }
    return nullptr;
  }

  void DynamicSystem::addLink(Link *lnk) {
    if(lnk->getName().empty())
      throwError("A empty object name is not allowed!");
    if (getLink(lnk->getName(), false)) {
      throwError("DynamicSystem can only comprise one Link by the name \"" + lnk->getName() + "\"!");
      assert(getLink(lnk->getName(),false) == nullptr);
    }

    link.push_back(lnk);
    lnk->setParent(this);
  }

  void DynamicSystem::addConstraint(Constraint *crt) {
    if(crt->getName().empty())
      throwError("A empty object name is not allowed!");
    if (getConstraint(crt->getName(), false)) {
      throwError("DynamicSystem can only comprise one Constraint by the name \"" + crt->getName() + "\"!");
      assert(getConstraint(crt->getName(),false) == nullptr);
    }

    constraint.push_back(crt);
    crt->setParent(this);
  }

  void DynamicSystem::addInverseKineticsLink(Link *lnk) {
    inverseKineticsLink.push_back(lnk);
    lnk->setParent(this);
  }

  Link* DynamicSystem::getLink(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < link.size(); i++) {
      if (link[i]->getName() == name)
        return link[i];
    }
    if (check) {
      if (!(i < link.size()))
        throwError("DynamicSystem comprises no Link \"" + name + "\"!");
      assert(i < link.size());
    }
    return nullptr;
  }

  Constraint* DynamicSystem::getConstraint(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < constraint.size(); i++) {
      if (constraint[i]->getName() == name)
        return constraint[i];
    }
    if (check) {
      if (!(i < constraint.size()))
        throwError("DynamicSystem comprises no Constraint \"" + name + "\"!");
      assert(i < constraint.size());
    }
    return nullptr;
  }

  void DynamicSystem::addModel(ModellingInterface *model_) {
    if(model_->getName().empty())
      throwError("A empty object name is not allowed!");
    if (getModel(model_->getName(), false)) {
      throwError("DynamicSystem can only comprise one model by the name \"" + model_->getName() + "\"!");
      assert(getModel(model_->getName(),false) == nullptr);
    }
    model.push_back(model_);
    model_->setParent(this);
  }

  ModellingInterface* DynamicSystem::getModel(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < model.size(); i++) {
      if (model[i]->getName() == name)
        return model[i];
    }
    if (check) {
      if (!(i < model.size()))
        throwError("DynamicSystem comprises no model \"" + name + "\"!");
      assert(i < model.size());
    }
    return nullptr;
  }

  void DynamicSystem::addGroup(DynamicSystem *sys) {
    if(sys->getName().empty())
      throwError("A empty object name is not allowed!");
    if (getGroup(sys->getName(), false)) {
      throwError("DynamicSystem can only comprise one DynamicSystem by the name \"" + sys->getName() + "\"!");
      assert(getGroup(sys->getName(),false) == nullptr);
    }
    dynamicsystem.push_back(sys);
    sys->setParent(this);
  }

  void DynamicSystem::addObject(Object *obj) {
    if(obj->getName().empty())
      throwError("A empty object name is not allowed!");
    if (getObject(obj->getName(), false)) {
      throwError("DynamicSystem can only comprise one Object by the name \"" + obj->getName() + "\"!");
      assert(getObject(obj->getName(),false) == nullptr);
    }
    object.push_back(obj);
    obj->setParent(this);
  }

  Element * DynamicSystem::getChildByContainerAndName(const std::string &container, const std::string &name) const {
    if (container == "Object")
      return getObject(name);
    else if (container == "Link")
      return getLink(name);
    else if (container == "Constraint")
      return getConstraint(name);
    else if (container == "Group")
      return getGroup(name);
    else if (container == "Frame")
      return getFrame(name);
    else if (container == "Contour")
      return getContour(name);
    else if (container == "Observer")
      return getObserver(name);
    else
      throwError("Unknown container '"+container+"'.");
  }

  Observer* DynamicSystem::getObserver(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < observer.size(); i++) {
      if (observer[i]->getName() == name)
        return observer[i];
    }
    if (check) {
      if (!(i < observer.size()))
        throwError("DynamicSystem comprises no Observer \"" + name + "\"!");
      assert(i < observer.size());
    }
    return nullptr;
  }

  void DynamicSystem::addObserver(Observer *ele) {
    if(ele->getName().empty())
      throwError("A empty object name is not allowed!");
    if (getObserver(ele->getName(), false)) {
      throwError("DynamicSystem can only comprise one Observer by the name \"" + ele->getName() + "\"!");
      assert(getObserver(ele->getName(),false) == nullptr);
    }
    observer.push_back(ele);
    ele->setParent(this);
  }

  void DynamicSystem::updatecorr(int j) {

    for (auto & i : linkSetValuedActive)
      i->updatecorr(j);
  }
  
  void DynamicSystem::updatecorrRef(fmatvec::Vec &ref) {
    corr.ref(ref, RangeV(corrInd, corrInd + corrSize - 1));

    for (auto & i : linkSetValued)
      (*i).updatecorrRef(ref);
  }

  void DynamicSystem::calccorrSize(int j) {
    corrSize = 0;

    for (auto & i : linkSetValued) {
      i->calccorrSize(j);
      i->setcorrInd(corrSize);
      corrSize += i->getcorrSize();
    }
  }

  void DynamicSystem::checkRoot() {

    for (auto & i : linkWithStopVector)
      i->checkRoot();
  }

  void DynamicSystem::resetUpToDate() {
    for (auto & i : dynamicsystem)
      i->resetUpToDate();
    for (auto & i : object)
      i->resetUpToDate();
    for (auto & i : link)
      i->resetUpToDate();
    for (auto & i : constraint)
      i->resetUpToDate();
    for (auto & i : observer)
      i->resetUpToDate();
    for (auto & i : inverseKineticsLink)
      i->resetUpToDate();
  }

  const fmatvec::Mat& DynamicSystem::getT(bool check) const {
    assert((not check) or (not ds->getUpdateT()));
    return T;
  }

  const fmatvec::Vec& DynamicSystem::geth(int i, bool check) const {
    assert((not check) or (not ds->getUpdateh(i)));
    return h[i];
  }

  const fmatvec::SymMat& DynamicSystem::getM(bool check) const {
    assert((not check) or (not ds->getUpdateM()));
    return M;
  }

  const fmatvec::SymMat& DynamicSystem::getLLM(bool check) const {
    assert((not check) or (not ds->getUpdateLLM()));
    return LLM;
  }

  const fmatvec::Mat& DynamicSystem::getW(int i, bool check) const {
    assert((not check) or (not ds->getUpdateW(i)));
    return W[i];
  }

  const fmatvec::Mat& DynamicSystem::getV(int i, bool check) const {
    assert((not check) or (not ds->getUpdateV(i)));
    return V[i];
  }

  const fmatvec::Vec& DynamicSystem::getg(bool check) const {
    assert((not check) or (not ds->getUpdateg()));
    return g;
  }

  const fmatvec::Vec& DynamicSystem::getgd(bool check) const {
    assert((not check) or (not ds->getUpdategd()));
    return gd;
  }

  const fmatvec::Vec& DynamicSystem::getla(bool check) const {
    assert((not check) or (not ds->getUpdatela()));
    return la;
  }

  const fmatvec::Vec& DynamicSystem::getLa(bool check) const {
    assert((not check) or (not ds->getUpdateLa()));
    return La;
  }

  const fmatvec::Vec& DynamicSystem::getdq(bool check) const {
    assert((not check) or (not ds->getUpdatedq()));
    return dq;
  }

  const fmatvec::Vec& DynamicSystem::getdu(bool check) const {
    assert((not check) or (not ds->getUpdatedu()));
    return du;
  }

  const fmatvec::Vec& DynamicSystem::getdx(bool check) const {
    assert((not check) or (not ds->getUpdatedx()));
    return dx;
  }

  const fmatvec::Vec& DynamicSystem::getqd(bool check) const {
    assert((not check) or (not ds->getUpdatezd()));
    return qd;
  }

  const fmatvec::Vec& DynamicSystem::getud(bool check) const {
    assert((not check) or (not ds->getUpdatezd()));
    return ud;
  }

  const fmatvec::Vec& DynamicSystem::getxd(bool check) const {
    assert((not check) or (not ds->getUpdatezd()));
    return xd;
  }

  fmatvec::SymMat& DynamicSystem::getLLM(bool check) {
    assert((not check) or (not ds->getUpdateLLM()));
    return LLM;
  }

  fmatvec::Mat& DynamicSystem::getW(int i, bool check) {
    assert((not check) or (not ds->getUpdateW(i)));
    return W[i];
  }

  fmatvec::Vec& DynamicSystem::getla(bool check) {
    assert((not check) or (not ds->getUpdatela()));
    return la;
  }

  fmatvec::Vec& DynamicSystem::getLa(bool check) {
    assert((not check) or (not ds->getUpdateLa()));
    return La;
  }

  fmatvec::Vec& DynamicSystem::getdq(bool check) {
    assert((not check) or (not ds->getUpdatedq()));
    return dq;
  }

  fmatvec::Vec& DynamicSystem::getdu(bool check) {
    assert((not check) or (not ds->getUpdatedu()));
    return du;
  }

  fmatvec::Vec& DynamicSystem::getdx(bool check) {
    assert((not check) or (not ds->getUpdatedx()));
    return dx;
  }

  const Mat& DynamicSystem::evalT() {
    if(ds->getUpdateT()) ds->updateT();
    return T;
  }

  const Vec& DynamicSystem::evalh(int i) {
    if(ds->getUpdateh(i)) ds->updateh(i);
    return h[i];
  }

  const Vec& DynamicSystem::evalr(int i) {
    if(ds->getUpdater(i)) ds->updater(i);
    return r[i];
  }

  const Mat& DynamicSystem::evalJrla(int i) {
    if(ds->getUpdateJrla(i)) ds->updateJrla(i);
    return Jrla[i];
  }

  const Vec& DynamicSystem::evalrdt() {
    if(ds->getUpdaterdt()) ds->updaterdt();
    return rdt;
  }

  const SymMat& DynamicSystem::evalM() {
    if(ds->getUpdateM()) ds->updateM();
    return M;
  }

  const SymMat& DynamicSystem::evalLLM() {
    if(ds->getUpdateLLM()) ds->updateLLM();
    return LLM;
  }

  const Mat& DynamicSystem::evalW(int i) {
    if(ds->getUpdateW(i)) ds->updateW(i);
    return W[i];
  }

  const Mat& DynamicSystem::evalV(int i) {
    if(ds->getUpdateV(i)) ds->updateV(i);
    return V[i];
  }

  const Vec& DynamicSystem::evalwb() {
    if(ds->getUpdatewb()) ds->updatewb();
    return wb;
  }

  const Vec& DynamicSystem::evalg() {
    if(ds->getUpdateg()) ds->updateg();
    return g;
  }

  const Vec& DynamicSystem::evalgd() {
    if(ds->getUpdategd()) ds->updategd();
    return gd;
  }

  const Mat& DynamicSystem::evalWInverseKinetics() {
    ds->updateWInverseKinetics();
    return WInverseKinetics;
  }

  const Mat& DynamicSystem::evalbInverseKinetics() {
    ds->updatebInverseKinetics();
    return bInverseKinetics;
  }

  void DynamicSystem::updateStateTable() {
    for(auto & i : dynamicsystem)
      i->updateStateTable();
    for(auto & i : object) {
      for(int j=0; j<i->getqSize(); j++)
        ds->getStateTable()[i->getqInd()+j] = StateTable(i->getPath(),'q',j);
      for(int j=0; j<i->getuSize(); j++)
        ds->getStateTable()[ds->getqSize()+i->getuInd()+j] = StateTable(i->getPath(),'u',j);
    }
    for (auto & i : link)
      for(int j=0; j<i->getxSize(); j++)
        ds->getStateTable()[ds->getqSize()+ds->getuSize()+i->getxInd()+j] = StateTable(i->getPath(),'x',j);
    for (auto & i : constraint)
      for(int j=0; j<i->getxSize(); j++)
        ds->getStateTable()[ds->getqSize()+ds->getuSize()+i->getxInd()+j] = StateTable(i->getPath(),'x',j);
    for (auto & i : link)
      for(int j=0; j<i->getisSize(); j++)
        ds->getStateTable()[ds->getqSize()+ds->getuSize()+ds->getxSize()+i->getisInd()+j] = StateTable(i->getPath(),'i',j);
    for (auto & i : constraint)
      for(int j=0; j<i->getisSize(); j++)
        ds->getStateTable()[ds->getqSize()+ds->getuSize()+ds->getxSize()+i->getisInd()+j] = StateTable(i->getPath(),'i',j);
    for (auto & i : observer)
      for(int j=0; j<i->getisSize(); j++)
        ds->getStateTable()[ds->getqSize()+ds->getuSize()+ds->getxSize()+i->getisInd()+j] = StateTable(i->getPath(),'i',j);
  }

}
