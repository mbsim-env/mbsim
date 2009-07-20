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
#include "mbsim/dynamic_system.h"
#include "mbsim/order_one_dynamics.h"
#include "mbsim/link.h"
#include "mbsim/contour.h"
#include "mbsim/object.h"
#include "mbsim/data_interface_base.h"
#include "mbsim/frame.h"
#include "mbsim/dynamic_system_solver.h"
#include "hdf5serie/fileserie.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/group.h"
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

#include<algorithm>

using namespace std;
using namespace fmatvec;

namespace MBSim {

  DynamicSystem::DynamicSystem(const string &name) : Element(name), parent(0), frameParent(0), PrPF(Vec(3,INIT,0.)), APF(SqrMat(3,EYE)), q0(0), u0(0), x0(0), qSize(0), qInd(0), xSize(0), xInd(0), gSize(0), gInd(0), gdSize(0), gdInd(0), laSize(0), laInd(0), rFactorSize(0), rFactorInd(0), svSize(0), svInd(0)
#ifdef HAVE_OPENMBVCPPINTERFACE                      
                                                     , openMBVGrp(0)
#endif
                                                     {
                                                       uSize[0] = 0;
                                                       uSize[1] = 0;
                                                       uInd[0] = 0;
                                                       uInd[1] = 0;
                                                       hSize[0] = 0;
                                                       hSize[1] = 0;
                                                       hInd[0] = 0;
                                                       hInd[1] = 0;

                                                       addFrame(new Frame("I"));

                                                       IrOF.push_back(Vec(3,INIT,0.));
                                                       AIF.push_back(SqrMat(3,EYE));
                                                     }

  DynamicSystem::~DynamicSystem() {
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      delete *i;
    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      delete *i;
    for(vector<OrderOneDynamics*>::iterator i = orderOneDynamics.begin(); i != orderOneDynamics.end(); ++i)
      delete *i;
    for(vector<DataInterfaceBase*>::iterator i = DIB.begin(); i != DIB.end(); ++i)
      delete *i;
    for(vector<Frame*>::iterator i = frame.begin(); i != frame.end(); ++i)
      delete *i;

#ifdef HAVE_OPENMBVCPPINTERFACE
    delete openMBVGrp;
#endif
  }

  void DynamicSystem::updateT(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateT(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateT(t);
  }

  void DynamicSystem::updateh(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateh(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateh(t);

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updateh(t);
  }

  void DynamicSystem::updatedhdz(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatedhdz(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatedhdz(t);

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updatedhdz(t);
  }

  void DynamicSystem::updateM(double t) {
#pragma omp parallel for schedule(static) shared(t) default(none)
    for(int i=0; i<(int)dynamicsystem.size(); i++) {
      try { dynamicsystem[i]->updateM(t); }
      catch(MBSimError error) { error.printExceptionMessage(); }
    }

#pragma omp parallel for num_threads(omp_get_max_threads()-1) schedule(dynamic, max(1,(int)object.size()/(10*omp_get_num_threads()))) shared(t) default(none) if((int)object.size()>30) 
    for(int i=0; i<(int)object.size(); i++) {
      try { object[i]->updateM(t); }
      catch(MBSimError error) { error.printExceptionMessage(); }
    }
  }

  void DynamicSystem::updatedq(double t, double dt) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updatedq(t,dt);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updatedq(t,dt);
  }

  void DynamicSystem::sethSize(int hSize_, int j) {
    hSize[j] = hSize_;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->sethSize((*i)->getuSize(j),j);
      (*i)->sethInd((*i)->getuInd(j),j);
    }

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->sethSize((*i)->getuSize(j),j);
      (*i)->sethInd((*i)->getuInd(j),j);
    }
  }

  void DynamicSystem::calcqSize() {
    qSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcqSize();
      (*i)->setqInd(qSize);
      qSize += (*i)->getqSize();
    }
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->calcqSize();
      (*i)->setqInd(qSize);
      qSize += (*i)->getqSize();
    }
  }

  void DynamicSystem::calcuSize(int j) {
    uSize[j] = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcuSize(j);
      (*i)->setuInd(uSize[j],j);
      uSize[j] += (*i)->getuSize(j);
    }
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->calcuSize(j);
      (*i)->setuInd(uSize[j],j);
      uSize[j] += (*i)->getuSize(j);
    }
  }

  int DynamicSystem::gethInd(DynamicSystem* sys, int i) {
    return (this == sys) ? 0 : ((parent == this) ? hInd[i] : hInd[i] + parent->gethInd(sys,i));
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Group* DynamicSystem::getOpenMBVGrp() { return openMBVGrp; }
#endif

  void DynamicSystem::updater(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updater(t);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).updater(t);
  }

  void DynamicSystem::updatewb(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updatewb(t);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).updatewb(t);
  }

  void DynamicSystem::updateW(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateW(t);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).updateW(t);
  }

  void DynamicSystem::updateV(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateV(t);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).updateV(t);
  }

  void DynamicSystem::updateg(double t) {
#pragma omp parallel for schedule(static) shared(t) default(none) 
    for(int i=0; i<(int)dynamicsystem.size(); i++) {
      try { dynamicsystem[i]->updateg(t); }
      catch(MBSimError error) { error.printExceptionMessage(); }
    }

#pragma omp parallel for schedule(dynamic, max(1,(int)linkSingleValued.size()/(10*omp_get_num_threads()))) shared(t) default(none) if((int)linkSingleValued.size()>30) 
    for(int i=0; i<(int)linkSingleValued.size(); i++) {
      try { linkSingleValued[i]->updateg(t); }
      catch(MBSimError error) { error.printExceptionMessage(); }
    }

#pragma omp parallel for schedule(dynamic, max(1,(int)linkSetValued.size()/(10*omp_get_num_threads()))) shared(t) default(none) if((int)linkSetValued.size()>30) 
    for(int i=0; i<(int)linkSetValued.size(); i++) {
      try { linkSetValued[i]->updateg(t); }
      catch(MBSimError error) { error.printExceptionMessage(); }
    }
  }

  void DynamicSystem::updategd(double t) {
#pragma omp parallel for schedule(static) shared(t) default(none)
    for(int i=0; i<(int)dynamicsystem.size(); i++) {
      try { dynamicsystem[i]->updategd(t); }
      catch(MBSimError error) { error.printExceptionMessage(); }
    }

#pragma omp parallel for schedule(static) shared(t) default(none) if((int)linkSingleValued.size()>30)
    for(int i=0; i<(int)linkSingleValued.size(); i++) {
      try { linkSingleValued[i]->updategd(t); }
      catch(MBSimError error) { error.printExceptionMessage(); }
    }

#pragma omp parallel for schedule(static) shared(t) default(none) if((int)linkSetValued.size()>30)
    for(int i=0; i<(int)linkSetValuedActive.size(); i++) { 
      try { linkSetValuedActive[i]->updategd(t); }
      catch(MBSimError error) { error.printExceptionMessage(); }
    }
  }

  void DynamicSystem::updatedx(double t, double dt) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatedx(t,dt);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatedx(t,dt);

    for(vector<OrderOneDynamics*>::iterator i = orderOneDynamics.begin(); i!= orderOneDynamics.end(); ++i) 
      (**i).updatedx(t,dt);

  }

  void DynamicSystem::updatexd(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatexd(t);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexd(t);

    for(vector<OrderOneDynamics*>::iterator i = orderOneDynamics.begin(); i!= orderOneDynamics.end(); ++i) 
      (**i).updatexd(t);
  }

  void DynamicSystem::updateStopVector(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateStopVector(t);
    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (*i)->updateStopVector(t); 
  }   

  void DynamicSystem::setDynamicSystemSolver(DynamicSystemSolver* sys) {
    Element::setDynamicSystemSolver(sys);
    for(unsigned i=0; i<dynamicsystem.size(); i++)
      dynamicsystem[i]->setDynamicSystemSolver(sys);

    for(unsigned i=0; i<object.size(); i++)
      object[i]->setDynamicSystemSolver(sys);

    for(unsigned i=0; i<link.size(); i++)
      link[i]->setDynamicSystemSolver(sys);

    for (unsigned i=0; i<orderOneDynamics.size(); i++)
      orderOneDynamics[i]->setDynamicSystemSolver(sys);
  }

  void DynamicSystem::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      for(unsigned i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->plot(t,dt);
      for(unsigned i=0; i<object.size(); i++)
        object[i]->plot(t,dt);
      for(unsigned i=0; i<link.size(); i++)
        link[i]->plot(t,dt);
      for(unsigned i=0; i<orderOneDynamics.size(); i++)
        orderOneDynamics[i]->plot(t,dt);
      for(unsigned i=0; i<frame.size(); i++)
        frame[i]->plot(t,dt);
    }
  }

  void DynamicSystem::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      for(unsigned i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->closePlot();
      for(unsigned i=0; i<object.size(); i++)
        object[i]->closePlot();
      for(unsigned i=0; i<link.size(); i++)
        link[i]->closePlot();
      for(unsigned i=0; i<orderOneDynamics.size(); i++)
        orderOneDynamics[i]->closePlot();
      for(unsigned i=0; i<frame.size(); i++)
        frame[i]->closePlot();

      if(getPlotFeature(separateFilePerDynamicSystem)==enabled)
        delete (H5::FileSerie*)plotGroup;
      else
        delete (H5::Group*)plotGroup;
    }
  }

  void DynamicSystem::init() {
    if(frameParent) {
      frame[0]->setPosition(frameParent->getPosition() + frameParent->getOrientation()*PrPF);
      frame[0]->setOrientation(frameParent->getOrientation()*APF);
    }
    else {
      if(parent) {
        frame[0]->setPosition(parent->getFrame("I")->getPosition() + parent->getFrame("I")->getOrientation()*PrPF);
        frame[0]->setOrientation(parent->getFrame("I")->getOrientation()*APF);
      }
      else {
        frame[0]->setPosition(getFrame("I")->getPosition() + getFrame("I")->getOrientation()*PrPF);
        frame[0]->setOrientation(getFrame("I")->getOrientation()*APF);
      }
    }
    for(unsigned int i=1; i<frame.size(); i++) { // kinematics of other frames can be updates from frame I 
      frame[i]->setPosition(frame[0]->getPosition() + frame[0]->getOrientation()*IrOF[i]);
      frame[i]->setOrientation(frame[0]->getOrientation()*AIF[i]);
    }
    for(unsigned int i=0; i<contour.size(); i++) { // kinematics of other contours can be updates from frame I
      contour[i]->setReferencePosition(frame[0]->getPosition() + frame[0]->getOrientation()*IrOC[i]);
      contour[i]->setReferenceOrientation(frame[0]->getOrientation()*AIC[i]);
      contour[i]->init();
    }
    for(unsigned int i=0; i<dynamicsystem.size(); i++) { // kinematics of other dynamicsystems can be updates from frame I
      dynamicsystem[i]->init();
    }

    for(unsigned i=0; i<object.size(); i++)
      object[i]->init();

    for(unsigned i=0; i<link.size(); i++)
      link[i]->init();

    for (unsigned i=0; i<orderOneDynamics.size(); i++)
      orderOneDynamics[i]->init();
  }

  void DynamicSystem::preinit() {
    for(unsigned int i=0; i<dynamicsystem.size(); i++) {
      dynamicsystem[i]->preinit();
    }

    for(unsigned i=0; i<object.size(); i++)
      object[i]->preinit();

    for(unsigned i=0; i<link.size(); i++)
      link[i]->preinit();

    for (unsigned i=0; i<orderOneDynamics.size(); i++)
      orderOneDynamics[i]->preinit();
  }

  int DynamicSystem::solveConstraintsFixpointSingle() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->solveConstraintsFixpointSingle(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveConstraintsFixpointSingle();

    return 0;
  }

  int DynamicSystem::solveImpactsFixpointSingle() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->solveImpactsFixpointSingle(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveImpactsFixpointSingle();

    return 0;
  }

  int DynamicSystem::solveConstraintsGaussSeidel() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->solveConstraintsGaussSeidel(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveConstraintsGaussSeidel();

    return 0;
  }

  int DynamicSystem::solveImpactsGaussSeidel() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->solveImpactsGaussSeidel(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveImpactsGaussSeidel();

    return 0;
  }

  int DynamicSystem::solveConstraintsRootFinding() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->solveConstraintsRootFinding(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveConstraintsRootFinding();

    return 0;
  }

  int DynamicSystem::solveImpactsRootFinding() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->solveImpactsRootFinding(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveImpactsRootFinding();

    return 0;
  }

  int DynamicSystem::jacobianConstraints() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->jacobianConstraints(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->jacobianConstraints();

    return 0;
  }

  int DynamicSystem::jacobianImpacts() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->jacobianImpacts(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->jacobianImpacts();

    return 0;
  }

  void DynamicSystem::checkConstraintsForTermination() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->checkConstraintsForTermination(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).checkConstraintsForTermination();
  }

  void DynamicSystem::checkImpactsForTermination() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->checkImpactsForTermination(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).checkImpactsForTermination();
  }

  void DynamicSystem::updaterFactors() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updaterFactors(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updaterFactors();
  }

  void DynamicSystem::initPlot() {
    if(parent)
      updatePlotFeatures(parent);

    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(separateFilePerDynamicSystem)==enabled) {
        // create symbolic link in parent plot file if exist
        if(parent) H5Lcreate_external((getPath()+".mbsim.h5").c_str(), "/",
            parent->getPlotGroup()->getId(), name.c_str(),
            H5P_DEFAULT, H5P_DEFAULT);
        // create new plot file (cast needed because of the inadequacy of the HDF5 C++ interface?)
        plotGroup=(H5::Group*)new H5::FileSerie(getPath()+".mbsim.h5", H5F_ACC_TRUNC);
      }
      else
        plotGroup=new H5::Group(parent->getPlotGroup()->createGroup(name));

      H5::SimpleAttribute<string>::setData(*plotGroup, "Description", "Object of class: "+getType());
      plotVectorSerie=NULL;

#ifdef HAVE_OPENMBVCPPINTERFACE
      openMBVGrp=new OpenMBV::Group();
      openMBVGrp->setName(name);
      if(parent) parent->openMBVGrp->addObject(openMBVGrp);
      if(getPlotFeature(separateFilePerDynamicSystem)==enabled)
        openMBVGrp->setSeparateFile(true);
#endif

      for(unsigned i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->initPlot();
      for(unsigned i=0; i<object.size(); i++)
        object[i]->initPlot();
      for(unsigned i=0; i<link.size(); i++)
        link[i]->initPlot();
      for(unsigned i=0; i<orderOneDynamics.size(); i++)
        orderOneDynamics[i]->initPlot();
      for(unsigned i=0; i<frame.size(); i++)
        frame[i]->initPlot();
    }
    plotGroup->flush(H5F_SCOPE_GLOBAL);
  }

  Frame* DynamicSystem::getFrame(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<frame.size(); i++) {
      if(frame[i]->getName() == name)
        return frame[i];
    }
    if(check) {
      if(!(i<frame.size())) cout << "ERROR (DynamicSystem:getFrame): The object " << this->name <<" comprises no frame " << name << "!" << endl; 
      assert(i<frame.size());
    }
    return NULL;
  }

  Contour* DynamicSystem::getContour(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<contour.size(); i++) {
      if(contour[i]->getName() == name)
        return contour[i];
    }
    if(check) {
      if(!(i<contour.size())) cout << "ERROR (DynamicSystem:getContour): The object " << this->name <<" comprises no contour " << name << "!" << endl; 
      assert(i<contour.size());
    }
    return NULL;
  }

  void DynamicSystem::updateqRef(const Vec &qParent) {
    q >> qParent(qInd,qInd+qSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateqRef(q);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateqRef(q);
  }

  void DynamicSystem::updateqdRef(const Vec &qdParent) {
    qd >> qdParent(qInd,qInd+qSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateqdRef(qd);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateqdRef(qd);
  }

  void DynamicSystem::updateuRef(const Vec &uParent) {
    u >> uParent(uInd[0],uInd[0]+uSize[0]-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateuRef(u);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateuRef(u);
  }

  void DynamicSystem::updateudRef(const Vec &udParent) {
    ud >> udParent(uInd[0],uInd[0]+uSize[0]-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateudRef(ud);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateudRef(ud);
  }

  void DynamicSystem::updatexRef(const Vec &xParent) {
    x >> xParent(xInd,xInd+xSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatexRef(x);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexRef(x);

    for(vector<OrderOneDynamics*>::iterator i = orderOneDynamics.begin(); i!= orderOneDynamics.end(); ++i) 
      (**i).updatexRef(x);
  }

  void DynamicSystem::updatexdRef(const Vec &xdParent) {
    xd >> xdParent(xInd,xInd+xSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatexdRef(xd);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexdRef(xd);

    for(vector<OrderOneDynamics*>::iterator i = orderOneDynamics.begin(); i!= orderOneDynamics.end(); ++i) 
      (**i).updatexdRef(xd);
  }

  void DynamicSystem::updatehRef(const Vec &hParent, const Vec &hObjectParent, const Vec &hLinkParent, int j) {
    h.resize() >> hParent(hInd[j],hInd[j]+hSize[j]-1);
    hObject.resize() >> hObjectParent(hInd[j],hInd[j]+hSize[j]-1);
    hLink.resize() >> hLinkParent(hInd[j],hInd[j]+hSize[j]-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatehRef(h,hObject,hLink,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatehRef(h,hObject,j);

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updatehRef(h,hLink);
  }

  void DynamicSystem::updatedhdqRef(const Mat &dhdqObjectParent, const Mat &dhdqLinkParent, int j) {
    dhdqObject.resize() >> dhdqObjectParent(Index(hInd[j],hInd[j]+hSize[j]-1),Index(qInd,qInd+qSize-1));
    dhdqLink.  resize() >> dhdqLinkParent  (Index(hInd[j],hInd[j]+hSize[j]-1),Index(qInd,qInd+qSize-1));

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updatedhdqRef(dhdqObject,dhdqLink,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatedhdqRef(dhdqObject,j);

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updatedhdqRef(dhdqLink);
  }
  
  void DynamicSystem::updatedhduRef(const SqrMat &dhduObjectParent, const SqrMat &dhduLinkParent, int j) {
    dhduObject.resize() >> dhduObjectParent(Index(hInd[j],hInd[j]+hSize[j]-1),Index(uInd[0],uInd[0]+uSize[0]-1));
    dhduLink.  resize() >> dhduLinkParent  (Index(hInd[j],hInd[j]+hSize[j]-1),Index(uInd[0],uInd[0]+uSize[0]-1));

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updatedhduRef(dhduObject,dhduLink,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatedhduRef(dhduObject,j);

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updatedhduRef(dhduLink);
  }
  
  void DynamicSystem::updatedhdtRef(const Vec &dhdtObjectParent, const Vec &dhdtLinkParent, int j) {
    dhdtObject.resize() >> dhdtObjectParent(hInd[j],hInd[j]+hSize[j]-1);
    dhdtLink.  resize() >> dhdtLinkParent  (hInd[j],hInd[j]+hSize[j]-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updatedhdtRef(dhdtObject,dhdtLink,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatedhdtRef(dhdtObject,j);

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updatedhdtRef(dhdtLink);
  }

  void DynamicSystem::updatefRef(const Vec &fParent) {
    f >> fParent(xInd,xInd+xSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatefRef(f);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatefRef(f);
  }

  void DynamicSystem::updaterRef(const Vec &rParent) {
    r >> rParent(hInd[0],hInd[0]+hSize[0]-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updaterRef(r);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updaterRef(r);

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updaterRef(r);
  }

  void DynamicSystem::updateTRef(const Mat& TParent) {
    T >> TParent(Index(qInd,qInd+qSize-1),Index(uInd[0],uInd[0]+uSize[0]-1));

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateTRef(T);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateTRef(T);
  }

  void DynamicSystem::updateMRef(const SymMat& MParent, int j) {
    M.resize() >> MParent(Index(hInd[j],hInd[j]+hSize[j]-1));

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateMRef(M,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateMRef(M,j);
  }

  void DynamicSystem::updateLLMRef(const SymMat& LLMParent, int j) {
    LLM.resize() >> LLMParent(Index(hInd[j],hInd[j]+hSize[j]-1));

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateLLMRef(LLM,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateLLMRef(LLM,j);
  }

  void DynamicSystem::updategRef(const Vec& gParent) {
    g.resize() >> gParent(gInd,gInd+gSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updategRef(g);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updategRef(g);
  }

  void DynamicSystem::updategdRef(const Vec& gdParent) {
    gd.resize() >> gdParent(gdInd,gdInd+gdSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updategdRef(gd);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updategdRef(gd);
  }

  void DynamicSystem::updatelaRef(const Vec &laParent) {
    la.resize() >> laParent(laInd,laInd+laSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updatelaRef(la);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updatelaRef(la);
  }

  void DynamicSystem::updatewbRef(const Vec &wbParent) {
    wb.resize() >> wbParent(laInd,laInd+laSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updatewbRef(wb);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updatewbRef(wb);
  }

  void DynamicSystem::updateWRef(const Mat &WParent, int j) {
    W.resize() >> WParent(Index(hInd[j],hInd[j]+hSize[j]-1),Index(laInd,laInd+laSize-1));

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateWRef(W,j);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updateWRef(W,j);
  }

  void DynamicSystem::updateVRef(const Mat &VParent, int j) {
    V.resize() >> VParent(Index(hInd[j],hInd[j]+hSize[j]-1),Index(laInd,laInd+laSize-1));

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateVRef(V,j);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updateVRef(V,j);
  }

  void DynamicSystem::updatesvRef(const Vec &svParent) {
    sv >> svParent(svInd,svInd+svSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updatesvRef(sv);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatesvRef(sv);
  }

  void DynamicSystem::updatejsvRef(const Vector<int> &jsvParent) {
    jsv >> jsvParent(svInd,svInd+svSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updatejsvRef(jsv);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatejsvRef(jsv);
  }

  void DynamicSystem::updateresRef(const Vec &resParent) {
    res.resize() >> resParent(laInd,laInd+laSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateresRef(res);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updateresRef(res);
  }

  void DynamicSystem::updaterFactorRef(const Vec &rFactorParent) {
    rFactor.resize() >> rFactorParent(rFactorInd,rFactorInd+rFactorSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updaterFactorRef(rFactor);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updaterFactorRef(rFactor);
  }

  void DynamicSystem::initz() {
    q = q0;
    u = u0;
    x = x0;
    for(unsigned i=0; i<dynamicsystem.size(); i++)
      dynamicsystem[i]->initz();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->initz();
    for(unsigned i=0; i<link.size(); i++)
      link[i]->initz();
    for(unsigned i=0; i<orderOneDynamics.size(); i++)
      orderOneDynamics[i]->initz();
  }

  void DynamicSystem::buildListOfObjects(vector<Object*> &obj, bool recursive) {
    for(unsigned int i=0; i<object.size(); i++)
      obj.push_back(object[i]);
    if(recursive)
      for(unsigned int i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->buildListOfObjects(obj,recursive);
  }

  void DynamicSystem::buildListOfLinks(vector<Link*> &lnk, bool recursive) {
    for(unsigned int i=0; i<link.size(); i++)
      lnk.push_back(link[i]);
    if(recursive)
      for(unsigned int i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->buildListOfLinks(lnk,recursive);
  }

  void DynamicSystem::buildListOfOrderOneDynamics(vector<OrderOneDynamics*> &ood, bool recursive) {
    for(unsigned int i=0; i<orderOneDynamics.size(); i++)
      ood.push_back(orderOneDynamics[i]);
    if(recursive)
      for(unsigned int i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->buildListOfOrderOneDynamics(ood,recursive);
  }

  void DynamicSystem::buildListOfModels(std::vector<ModellingInterface*> &modelList, bool recursive) {
    for(unsigned int i=0; i<model.size(); i++)
      modelList.push_back(model[i]);
    if(recursive)
      for(unsigned int i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->buildListOfModels(modelList,recursive);
  }

  void DynamicSystem::updateCondition() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateCondition();

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (*i)->updateCondition();
  }

  void DynamicSystem::resizeJacobians(int j) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->resizeJacobians(j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->resizeJacobians(j);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (*i)->resizeJacobians(j);
  }

  void DynamicSystem::checkForConstraints() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->checkForConstraints();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->checkForConstraints();
  }

  void DynamicSystem::setUpLinks() {
    for(unsigned int i=0; i<dynamicsystem.size(); i++) 
      dynamicsystem[i]->setUpLinks();

    for(unsigned int i=0; i<link.size(); i++) {
      if(link[i]->isSetValued()) {
        linkSetValued.push_back(link[i]);
        linkSetValuedActive.push_back(link[i]);
      }
      else 
        linkSingleValued.push_back(link[i]);
    }
  }

  bool DynamicSystem::gActiveChanged() {
    bool changed = false;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      if ((*i)->gActiveChanged())
        changed = true;

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      if ((*i)->gActiveChanged())
        changed = true;

    return changed;
  }

  void DynamicSystem::calcxSize() {
    xSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcxSize();
      (*i)->setxInd(xSize);
      xSize += (*i)->getxSize();
    }

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) {
      (*i)->calcxSize();
      (*i)->setxInd(xSize);
      xSize += (*i)->getxSize();
    }

    for(vector<OrderOneDynamics*>::iterator i = orderOneDynamics.begin(); i != orderOneDynamics.end(); ++i) {
      (*i)->calcxSize();
      (*i)->setxInd(xSize);
      xSize += (*i)->getxSize();
    }
  }

  void DynamicSystem::calcsvSize() {
    svSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcsvSize();
      (*i)->setsvInd(svSize);
      svSize += (*i)->getsvSize();
    }
    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) {
      (*i)->calcsvSize();
      (*i)->setsvInd(svSize);
      svSize += (*i)->getsvSize();
    }
  }

  void DynamicSystem::calclaSize() {
    laSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calclaSize();
      (*i)->setlaInd(laSize);
      laSize += (*i)->getlaSize();
    }

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (*i)->calclaSize();
      (*i)->setlaInd(laSize);
      laSize += (*i)->getlaSize();
    }
  }

  void DynamicSystem::calclaSizeForActiveg() {
    laSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calclaSizeForActiveg();
      (*i)->setlaInd(laSize);
      laSize += (*i)->getlaSize();
    }

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (*i)->calclaSizeForActiveg();
      (*i)->setlaInd(laSize);
      laSize += (*i)->getlaSize();
    }
  }

  void DynamicSystem::calcgSize() {
    gSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcgSize();
      (*i)->setgInd(gSize);
      gSize += (*i)->getgSize();
    }

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calcgSize();
      (*i)->setgInd(gSize);
      gSize += (*i)->getgSize();
    }
  }

  void DynamicSystem::calcgSizeActive() {
    gSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcgSizeActive();
      (*i)->setgInd(gSize);
      gSize += (*i)->getgSize();
    }

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (*i)->calcgSizeActive();
      (*i)->setgInd(gSize);
      gSize += (*i)->getgSize();
    }
  }

  void DynamicSystem::calcgdSize() {
    gdSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcgdSize();
      (*i)->setgdInd(gdSize);
      gdSize += (*i)->getgdSize();
    }

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calcgdSize();
      (*i)->setgdInd(gdSize);
      gdSize += (*i)->getgdSize();
    }
  }

  void DynamicSystem::calcgdSizeActive() {
    gdSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcgdSizeActive();
      (*i)->setgdInd(gdSize);
      gdSize += (*i)->getgdSize();
    }

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (*i)->calcgdSizeActive();
      (*i)->setgdInd(gdSize);
      gdSize += (*i)->getgdSize();
    }
  }

  void DynamicSystem::calcrFactorSize() {
    rFactorSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcrFactorSize();
      (*i)->setrFactorInd(rFactorSize);
      rFactorSize += (*i)->getrFactorSize();
    }

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (*i)->calcrFactorSize();
      (*i)->setrFactorInd(rFactorSize);
      rFactorSize += (*i)->getrFactorSize();
    }
  }

  void DynamicSystem::checkActiveLinks() {

    linkSetValuedActive.clear();

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->checkActiveLinks();

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      if((*i)->isActive()) {
        linkSetValuedActive.push_back(*i);
      }
    }
  }

  void DynamicSystem::checkActiveg() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->checkActiveg();

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (*i)->checkActiveg();
  }

  void DynamicSystem::checkActivegd() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->checkActivegd();

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->checkActivegd();
  }

  void DynamicSystem::checkActivegdn() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->checkActivegdn();

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->checkActivegdn();
  }

  void DynamicSystem::checkActivegdd() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->checkActivegdd();

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->checkActivegdd();
  }

  void DynamicSystem::checkAllgd() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->checkAllgd();

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->checkAllgd();
  }

  void DynamicSystem::setgdTol(double tol) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i!= dynamicsystem.end(); ++i)
      (**i).setgdTol(tol);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setgdTol(tol);
  }

  void DynamicSystem::setgddTol(double tol) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i!= dynamicsystem.end(); ++i)
      (**i).setgddTol(tol);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setgddTol(tol);
  }

  void DynamicSystem::setlaTol(double tol) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i!= dynamicsystem.end(); ++i)
      (**i).setlaTol(tol);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setlaTol(tol);
  }

  void DynamicSystem::setLaTol(double tol) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i!= dynamicsystem.end(); ++i)
      (**i).setLaTol(tol);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setLaTol(tol);
  }

  void DynamicSystem::setrMax(double rMax) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i!= dynamicsystem.end(); ++i)
      (**i).setrMax(rMax);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setrMax(rMax);
  }

  void DynamicSystem::setlaIndDS(int laIndParent) {
    int newlaInd = laInd + laIndParent;
    for(unsigned i=0; i<dynamicsystem.size(); i++)
      dynamicsystem[i]->setlaIndDS(newlaInd);

    for(unsigned i=0; i<link.size(); i++)
      link[i]->setlaIndDS(newlaInd);
  }

  void DynamicSystem::addFrame(Frame* cosy) {
    if(getFrame(cosy->getName(),false)) { 
      cout << "ERROR (DynamicSystem:addFrame): The DynamicSystem " << name << " can only comprise one Frame by the name " <<  cosy->getName() << "!" << endl;
      assert(getFrame(cosy->getName(),false)==NULL);
    }
    frame.push_back(cosy);
    cosy->setParent(this);
  }

  void DynamicSystem::addFrame(Frame* cosy, const Vec &RrRF, const SqrMat &ARF, const Frame* refFrame) {
    addFrame(cosy);

    int i = 0;
    if(refFrame) i = frameIndex(refFrame);

    IrOF.push_back(IrOF[i] + AIF[i]*RrRF);
    AIF.push_back(AIF[i]*ARF);
  }

  void DynamicSystem::addFrame(const string &str, const Vec &RrRF, const SqrMat &ARF, const Frame* refFrame) {
    addFrame(new Frame(str),RrRF,ARF,refFrame);
  }

  void DynamicSystem::addContour(Contour* contour_) {
    if(getContour(contour_->getName(),false)) { 
      cout << "ERROR (DynamicSystem:addContour): The DynamicSystem " << name << " can only comprise one Contour by the name " <<  contour_->getName() << "!" << endl;
      assert(getContour(contour_->getName(),false)==NULL);
    }
    contour.push_back(contour_);
    contour_->setParent(this);
  }

  void DynamicSystem::addContour(Contour* contour, const Vec &RrRC, const SqrMat &ARC, const Frame* refFrame) {
    addContour(contour);

    if(dynamic_cast<const Frame*>(refFrame)!=0) {
      int i = frameIndex(static_cast<const Frame*>(refFrame));
      IrOC.push_back(IrOF[i] + AIF[i]*RrRC);
      AIC.push_back(AIF[i]*ARC);
    }
    else {
      IrOC.push_back(RrRC);
      AIC.push_back(ARC);
    }
  }

  int DynamicSystem::frameIndex(const Frame *frame_) const {
    for(unsigned int i=0; i<frame.size(); i++) {
      if(frame_==frame[i])
        return i;
    }
    return -1;
  }

  DynamicSystem* DynamicSystem::getDynamicSystem(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<dynamicsystem.size(); i++) {
      if(dynamicsystem[i]->getName() == name)
        return dynamicsystem[i];
    }
    if(check){
      if(!(i<dynamicsystem.size())) cout << "ERROR (DynamicSystem:getDynamicSystem): The DynamicSystem " << this->name <<" comprises no DynamicSystem " << name << "!" << endl; 
      assert(i<dynamicsystem.size());
    }
    return NULL;
  }

  Object* DynamicSystem::getObject(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<object.size(); i++) {
      if(object[i]->getName() == name)
        return object[i];
    }
    if(check){
      if(!(i<object.size())) cout << "ERROR (DynamicSystem::getObject): The DynamicSystem " << this->name <<" comprises no Object " << name << "!" << endl; 
      assert(i<object.size());
    }
    return NULL;
  }

  void DynamicSystem::addLink(Link *lnk) {
    if(getLink(lnk->getName(),false)) {
      cout << "ERROR (DynamicSystem: addLink): The DynamicSystem " << name << " can only comprise one Link by the name " <<  lnk->getName() << "!" << endl;
      assert(getLink(lnk->getName(),false) == NULL);
    }

    link.push_back(lnk);
    lnk->setParent(this);
  }

  Link* DynamicSystem::getLink(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<link.size(); i++) {
      if(link[i]->getName() == name)
        return link[i];
    }
    if(check){
      if(!(i<link.size())) cout << "ERROR (DynamicSystem::getLink): The DynamicSystem " << this->name <<" comprises no Link " << name << "!" << endl; 
      assert(i<link.size());
    }
    return NULL;
  }

  OrderOneDynamics* DynamicSystem::getOrderOneDynamics(const string &name,bool check) {
    unsigned int i;
    for(i=0; i<orderOneDynamics.size(); i++) {
      if(orderOneDynamics[i]->getName() == name) return orderOneDynamics[i];
    }
    if(check) {
      if(!(i<orderOneDynamics.size())) cout << "ERROR (DynamicSystem::getOrderOneDynamics): The DynamicSystem " << this->name <<" comprises no OrderOneDynamics " << name << "!" << endl; 
      assert(i<orderOneDynamics.size());
    }
    return NULL; 
  }    

  void DynamicSystem::addOrderOneDynamics(OrderOneDynamics *ood_) {
    if(getOrderOneDynamics(ood_->getName(),false)) {
      cout << "ERROR (DynamicSystem::addOrderOneDynamics): The DynamicSystem " << name << " can only comprise one OrderOneDynamics by the name " <<  ood_->getName() << "!" << endl;
      assert(getOrderOneDynamics(ood_->getName(),false) == NULL);
    }
    orderOneDynamics.push_back(ood_);
    ood_->setParent(this);
  }

  void DynamicSystem::addDataInterfaceBase(DataInterfaceBase* dib_) {
    if(getDataInterfaceBase(dib_->getName(),false)) {
      cout << "ERROR (DynamicSystem::addDataInterfaceBase): The DynamicSystem " << name << " can only comprise one DataInterfaceBase by the name " <<  dib_->getName() << "!" << endl;
      assert(getDataInterfaceBase(dib_->getName(),false) == NULL);
    }
    DIB.push_back(dib_);
  }

  DataInterfaceBase* DynamicSystem::getDataInterfaceBase(const string &name_, bool check) {
    unsigned int i;
    for(i=0; i<DIB.size(); i++) {
      if(DIB[i]->getName() == name_ || DIB[i]->getName() == name_+".SigOut")
        return DIB[i];
    }
    if(check){
      if(!(i<DIB.size())) cout << "ERROR (DynamicSystem::getDataInterfaceBase): The DynamicSystem " << name <<" comprises no DataInterfaceBase " << name_ << "!" << endl; 
      assert(i<DIB.size());
    } 
    return NULL;
  }    

  void DynamicSystem::addModel(ModellingInterface *model_) {
    if(getModel(model_->getName(),false)) {
      cout << "ERROR (DynamicSystem::addModell): The DynamicSystem " << name << " can only comprise one model by the name " <<  model_->getName() << "!" << endl;
      assert(getModel(model_->getName(),false) == NULL); 
    }
    model.push_back(model_);
  }

  ModellingInterface* DynamicSystem::getModel(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<model.size(); i++) {
      if(model[i]->getName() == name)
        return model[i];
    }
    if(check){
      if(!(i<model.size())) cout << "ERROR (DynamicSystem::getModell): The DynamicSystem " << name <<" comprises no model " << name << "!" << endl; 
      assert(i<model.size());
    }
    return NULL;
  }

  void DynamicSystem::addDynamicSystem(DynamicSystem *sys) {
    if(getDynamicSystem(sys->getName(),false)) {
      cout << "ERROR (DynamicSystem::addDynamicSystem): The DynamicSystem " << name << " can only comprise one DynamicSystem by the name " <<  sys->getName() << "!" << endl;
      assert(getDynamicSystem(sys->getName(),false) == NULL); 
    }
    dynamicsystem.push_back(sys);
    sys->setParent(this);
  }

  void DynamicSystem::addObject(Object *obj) {
    if(getObject(obj->getName(),false)) {
      cout << "ERROR (DynamicSystem::addObject): The DynamicSystem " << name << " can only comprise one Object by the name " <<  obj->getName() << "!" << endl;
      assert(getObject(obj->getName(),false) == NULL); 
    }
    object.push_back(obj);
    obj->setParent(this);
  }

  Frame *DynamicSystem::getFrameByPath(std::string path) {
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
    else if(firstPart.substr(0,6)=="Frame[")
      return getFrame(firstPart.substr(6,firstPart.find(']')-6));
    else if(firstPart.substr(0,7)=="Object[")
      return getObject(firstPart.substr(7,firstPart.find(']')-7))->getFrameByPath(restPart);
    else if(firstPart.substr(0,14)=="DynamicSystem[")
      return getDynamicSystem(firstPart.substr(14,firstPart.find(']')-14))->getFrameByPath(restPart);
    else
      return 0;
  }

  Contour *DynamicSystem::getContourByPath(std::string path) {
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
    else if(firstPart.substr(0,7)=="Object[")
      return getObject(firstPart.substr(7,firstPart.find(']')-7))->getContourByPath(restPart);
    else if(firstPart.substr(0,8)=="Contour[")
      return getContour(firstPart.substr(8,firstPart.find(']')-8));
    else if(firstPart.substr(0,14)=="DynamicSystem[")
      return getDynamicSystem(firstPart.substr(14,firstPart.find(']')-14))->getContourByPath(restPart);
    else
      return 0;
  }

}

