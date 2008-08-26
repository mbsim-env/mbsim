/* Copyright (C) 2004-2006  Martin Förg, Roland Zander

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
 *   rzander@users.berlios.de
 *
 */
#include<config.h>
#include "multi_body_system.h"
#include "port.h"
#include "contour.h"
#include "link.h"
#include "extra_dynamic_interface.h"
#include "integrator.h"
#include "eps.h"
#include "function.h"
#include "nonlinear_algebra.h"
#ifndef MINGW
#  include<sys/stat.h>
#else
#  include<io.h>
#  define mkdir(a,b) mkdir(a)
#endif

namespace MBSim {

  MultiBodySystem::MultiBodySystem() : Object("Default"), grav(3), gSize(0), gIndBilateral(0), gIndUnilateral(0), laSize(0), laIndBilateral(0), laIndUnilateral(0), rFactorSize(0), svSize(0), svInd(0), nHSLinksSetValuedUnilateralFixed(0),nHSLinksSetValuedBilateralFixed(0), nHSLinksSingleValuedFixed(0), checkGSize(true), limitGSize(500), maxIter(10000), highIter(1000), maxDampingSteps(3), lmParm(0.001), solver(FixedPointSingle), strategy(local), linAlg(LUDecomposition), stopIfNoConvergence(false), activeConstraintsChanged(true), dropContactInfo(false), useOldla(true), numJac(false), directoryName("Default"), preIntegrator(NULL) {} 

  MultiBodySystem::MultiBodySystem(const string &projectName) : Object(projectName), grav(3), gSize(0), gIndBilateral(0), gIndUnilateral(0), laSize(0), laIndBilateral(0), laIndUnilateral(0), rFactorSize(0), svSize(0), svInd(0), nHSLinksSetValuedUnilateralFixed(0),nHSLinksSetValuedBilateralFixed(0), nHSLinksSingleValuedFixed(0), checkGSize(true), limitGSize(500), maxIter(10000), highIter(1000), maxDampingSteps(3), lmParm(0.001), solver(FixedPointSingle), strategy(local), linAlg(LUDecomposition), stopIfNoConvergence(false), activeConstraintsChanged(true), dropContactInfo(false), useOldla(true), numJac(false), directoryName(projectName), preIntegrator(NULL) {}

  MultiBodySystem::~MultiBodySystem() {
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) delete *i;
    vector<Link*>::iterator iL;
    for(iL = links.begin(); iL != links.end(); ++iL) delete *iL;
    vector<ExtraDynamicInterface*>::iterator iEDI;
    for(iEDI = EDI.begin(); iEDI != EDI.end(); ++iEDI) delete *iEDI;
    vector<DataInterfaceBase*>::iterator iD;
    for(iD = DIBs.begin(); iD != DIBs.end(); ++iD) delete *iD;
    vector<HitSphereLink*>::iterator iHS;
    for(iHS =  HSLinks.begin(); iHS != HSLinks.end(); ++iHS) delete *iHS;
    if (preIntegrator) delete preIntegrator;
  }

  void MultiBodySystem::setGrav(const Vec& g) 
  {
    grav = g;
  }

  double MultiBodySystem::computePotentialEnergy()
  {
    double Vpot = 0.0;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) Vpot += (**i).computePotentialEnergy();

    vector<Link*>::iterator ic;
    for(ic = links.begin(); ic != links.end(); ++ic) Vpot += (**ic).computePotentialEnergy();
    return Vpot;
  }

  Object* MultiBodySystem::getObject(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<objects.size(); i++) {
      if(objects[i]->getName() == name) return objects[i];
    }
    for(i=0; i<objects.size(); i++) {
      if(objects[i]->getFullName() == name) return objects[i];
    }
    if(check){
      if(!(i<objects.size())) cout << "ERROR (MultiBodySystem:getObject): The MultiBodySystem " << this->name << " comprises no object " << name << "!" << endl; 
      assert(i<objects.size());
    }
    return NULL;
  }

  Link* MultiBodySystem::getLink(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<links.size(); i++) {
      if(links[i]->getName() == name) return links[i];
    }
    for(i=0; i<links.size(); i++) {
      if(links[i]->getFullName() == name) return links[i];
    }
    if(check){
      if(!(i<links.size())) cout << "ERROR (MultiBodySystem:getLink): The MultiBodySystem " << this->name << " comprises no link " << name << "!" << endl; 
      assert(i<links.size());
    }
    return NULL;
  }

  Port* MultiBodySystem::getPort(const string &name,bool check) {
    unsigned int i;
    for(i=0; i<port.size(); i++) {
      if(port[i]->getName() == name || port[i]->getFullName()== name) return port[i];
    }
    if(check){
      if(!(i<port.size())) cout << "ERROR (MultiBodySystem:getPort): The MultiBodySystem " << this->name <<" comprises no port " << name << "!" << endl; 
      assert(i<port.size());
    }
    return NULL;
  }

  Contour* MultiBodySystem::getContour(const string &name,bool check) {
    unsigned int i;
    for(i=0; i<contour.size(); i++) {
      if(contour[i]->getName() == name || contour[i]->getFullName()== name) return contour[i];
    }
    if(check){
      if(!(i<contour.size())) cout << "ERROR (MultiBodySystem:getContour): The MultiBodySystem " << this->name << " comprises no contour " << name << "!" << endl; 
      assert(i<contour.size());
    }
    return NULL;
  }

  ExtraDynamicInterface* MultiBodySystem::getEDI(const string &name,bool check) 
  {
    unsigned int i;
    for(i=0; i<EDI.size(); i++) {
      if(EDI[i]->getName() == name) return EDI[i];
    }
    for(i=0; i<EDI.size(); i++) {
      if(EDI[i]->getFullName() == name) return EDI[i];
    }
    if(check) {
      if(!(i<EDI.size())) cout << "ERROR (MultiBodySystem:getEDI): The MultiBodySystem " << this->name <<" comprises no EDI " << name << "!" << endl; 
      assert(i<EDI.size());
    }
    return NULL; 
  }

  DataInterfaceBase* MultiBodySystem::getDataInterfaceBase(const string &name_, bool check) {
    unsigned int i;
    for(i=0; i<DIBs.size(); i++) {
      if(DIBs[i]->getName() == name_ || DIBs[i]->getName()== fullName+"."+name_ || DIBs[i]->getName() == name_+".SigOut" || DIBs[i]->getName()== fullName+"."+name_+".SigOut") return DIBs[i];
    }
    if(check){
      if(!(i<DIBs.size())) cout << "ERROR (MultiBodySystem:getDataInterfaceBase): The MultiBodySystem " << name <<" comprises no DIB " << name_ << "!" << endl; 
      assert(i<DIBs.size());
    } 
    return NULL;
  } 

  Element* MultiBodySystem::getElement(const string &name) 
  {
    unsigned int i1;
    for(i1=0; i1<objects.size(); i1++) {
      if(objects[i1]->getName() == name) return (Element*)objects[i1];
    }
    for(i1=0; i1<objects.size(); i1++) {
      if(objects[i1]->getFullName() == name) return (Element*)objects[i1];
    }
    unsigned int i2;
    for(i2=0; i2<links.size(); i2++) {
      if(links[i2]->getName() == name) return (Element*)links[i2];
    }
    for(i2=0; i2<links.size(); i2++) {
      if(links[i2]->getFullName() == name) return (Element*)links[i2];
    }
    unsigned int i3;
    for(i3=0; i3<EDI.size(); i3++) {
      if(EDI[i3]->getName() == name) return (Element*)EDI[i3];
    }
    for(i3=0; i3<EDI.size(); i3++) {
      if(EDI[i3]->getFullName() == name) return (Element*)EDI[i3];
    }
    if(!(i1<objects.size())||!(i2<links.size())||!(i3<EDI.size())) cout << "Error: The MultiBodySystem " << this->name << " comprises no element " << name << "!" << endl; 
    assert(i1<objects.size()||i2<links.size()||!(i3<EDI.size()));
    return NULL;
  }

  void MultiBodySystem::addObject(Object *object) 
  {
    if(getObject(object->getFullName(),false)) {
      cout << "ERROR (MultiBodySystem:addObject): " << name << " can only comprise one Object by the name " <<  object->getFullName() << "!" << endl;
      assert(getObject(object->getFullName(),false) == NULL); 
    }
    objects.push_back(object);
    object->setMbs(this);
    object->setFullName(getFullName()+"."+object->getFullName());

  }

  void MultiBodySystem::addMbs(MultiBodySystem* mbs)
  {
    for(unsigned int i=0; i<mbs->port.size(); i++) Object::addPort(mbs->port[i]);
    for(unsigned int i=0; i<mbs->contour.size(); i++) Object::addContour(mbs->contour[i]);
    for(unsigned int i=0; i<mbs->objects.size(); i++) addObject(mbs->objects[i]);
    for(unsigned int i=0; i<mbs->links.size(); i++) addLink(mbs->links[i]);
    for(unsigned int i=0; i<mbs->EDI.size(); i++) addEDI(mbs->EDI[i]);
    for(unsigned int i=0; i<mbs->DIBs.size(); i++) addDataInterfaceBase(mbs->DIBs[i]);
  }

  void MultiBodySystem::addLink(Link *link) {
    if(getLink(link->getFullName(),false)) {
      cout << "ERROR (MultiBodySystem:addLink): " << name << " can only comprise one Link by the name " <<  link->getFullName() << "!" << endl;
      assert(getLink(link->getFullName(),false) == NULL);
    }
    links.push_back(link);
    link->setMbs(this);
    link->setFullName(getFullName()+"."+link->getFullName());

  }

  void MultiBodySystem::addPort(const string &name, const Vec &WrOP) {
    Port *port = new Port(name);
    addPort(port,WrOP);
  }

  void MultiBodySystem::addPort(Port* port, const Vec &WrOP) {
    Object::addPort(port);
    port->setWrOP(WrOP);
  }

  void MultiBodySystem::addContour(Contour* contour, const Vec &WrOP) {
    Object::addContour(contour);
    contour->setWrOP(WrOP);
  }

  void MultiBodySystem::addContour(Contour* contour, const Vec &WrOP, const SqrMat &AWC) {
    Object::addContour(contour);
    contour->setWrOP(WrOP);
    contour->setAWC(AWC);
  }

  void MultiBodySystem::addEDI(ExtraDynamicInterface *edi_) {
    if(getEDI(edi_->getFullName(),false)) {
      cout << "ERROR (MultiBodySystem:addEDI): " << name << " can only comprise one ExtraDynamicInterface by the name " <<  edi_->getFullName() << "!" << endl;
      assert(getEDI(edi_->getFullName(),false) == NULL);
    }
    EDI.push_back(edi_);
    edi_->setMbs(this);
    edi_->setFullName(getFullName()+"."+edi_->getFullName());
  }

  void MultiBodySystem::addDataInterfaceBase(DataInterfaceBase* dib_){
    if(getDataInterfaceBase(dib_->getName(),false)) {
      cout << "ERROR (MultiBodySystem:addDataInterfaceBase): " << name << " can only comprise one DataInterfaceBase by the name " <<  dib_->getName() << "!" << endl;
      assert(getDataInterfaceBase(dib_->getName(),false) == NULL);
    }
    DIBs.push_back(dib_);
    dib_->setName(getFullName()+"."+dib_->getName());
  }

  void MultiBodySystem::addElement(Element *element_) {
    MultiBodySystem* mbs_=dynamic_cast<MultiBodySystem*>(element_);
    if(mbs_) addMbs(mbs_);
    else {
      Object* object_=dynamic_cast<Object*>(element_);
      Link* link_=dynamic_cast<Link*>(element_);
      ExtraDynamicInterface* edi_=dynamic_cast<ExtraDynamicInterface*>(element_);
      if(object_) addObject(object_);
      else if(link_) addLink(link_);
      else if(edi_) addEDI(edi_);
      else{ cout << "ERROR (MultiBodySystem:addElement): MultiBodySystem: addElement(): No such type of Element to add!" << endl; throw 50;}
    }
  }   

  HitSphereLink* MultiBodySystem::getHitSphereLink(Object* obj0, Object* obj1) {
    for(vector<HitSphereLink*>::iterator hsl = HSLinks.begin();hsl < HSLinks.end();hsl++)
      if((*hsl)->getObject(0) == obj0 && (*hsl)->getObject(1) == obj1 || (*hsl)->getObject(0) == obj1 && (*hsl)->getObject(1) == obj0) return  (*hsl);

    HitSphereLink *HSLink = new HitSphereLink(); // create new hitsphere link if none is found
    HSLinks.push_back(HSLink);
    return HSLink;
  }

  void MultiBodySystem::init() 
  { 
    if(INFO) cout << endl << "Initialising MultiBodySystem " << fullName << " ......" << endl;
    setDirectory();

    if(INFO) cout << "  setting dimensions of ..." << endl;
    // Objects
    if(objects.size()>0 && INFO)  cout << "      Object parameters" << endl;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) {
      (*i)->calcSize();
      (*i)->setqInd(qSize);
      (*i)->setuInd(uSize);
      (*i)->setxInd(xSize);
      qSize += (*i)->getqSize();
      uSize += (*i)->getuSize();
      xSize += (*i)->getxSize();
    }

    if(links.size()>0 && INFO) cout << "      Link parameters" " (" << links.size() <<")"<< endl;


    int gSizeTmp, laSizeTmp;
    laSizeTmp = laSize;
    gSizeTmp = gSize;
    for(vector<Link*>::iterator il = links.begin(); il != links.end(); ++il) {
      (*il)->calcSize();
      if((*il)->isSetValued() && (*il)->isBilateral()) {
	(*il)->setgInd(gSize);
	(*il)->setlaInd(laSize);
	(*il)->setrFactorInd(rFactorSize);
	gSize += (*il)->getgSize();
	laSize += (*il)->getlaSize();
	rFactorSize += (*il)->getrFactorSize();
      }
      (*il)->setxInd(xSize);
      xSize += (*il)->getxSize();
      (*il)->setsvInd(svSize);
      svSize += (*il)->getsvSize();
    }
    gIndBilateral = Index(gSizeTmp, gSize-1);
    laIndBilateral= Index(laSizeTmp, laSize-1);

    gSizeTmp = gSize;
    laSizeTmp = laSize;
    for(vector<Link*>::iterator il = links.begin(); il != links.end(); ++il) {
      if((*il)->isSetValued() && (! (*il)->isBilateral())) {
	(*il)->setgInd(gSize);
	(*il)->setlaInd(laSize);
	(*il)->setrFactorInd(rFactorSize);
	gSize += (*il)->getgSize();
	laSize += (*il)->getlaSize();
	rFactorSize += (*il)->getrFactorSize();
      }
    }
    gIndUnilateral = Index(gSizeTmp, gSize-1);
    laIndUnilateral= Index(laSizeTmp, laSize-1);
    if(gSize) {
      cout << "           gSize : " << gSize;
      cout << " (" << gIndBilateral.end()-gIndBilateral.start() +1 <<" bilateral and ";
      cout << gIndUnilateral.end()-gIndUnilateral.start()+1 <<" unilateral)"<<endl;
    }
    if(laSize) {
      cout << "           laSize: " << laSize;
      cout << " (" << laIndBilateral.end()-laIndBilateral.start() +1 <<" bilateral and ";
      cout << laIndUnilateral.end()-laIndUnilateral.start()+1 <<" unilateral)"<<endl;
    }

    // EDIs
    if(EDI.size()>0 && INFO) cout << "      EDI parameters" << endl;

    vector<ExtraDynamicInterface*>::iterator iF;
    for(iF = EDI.begin(); iF!= EDI.end(); ++iF) {
      (*iF)->setxInd(xSize);
      xSize += (*iF)->getxSize();
    }

    // TODO memory problem with many contacts
    if(laSize>8000) laSize=8000;
    MParent.resize(getuSize());
    TParent.resize(getqSize(),getuSize());
    LLMParent.resize(getuSize());
    WParent.resize(getuSize(),getlaSize());
    GParent.resize(getlaSize());
    bParent.resize(getlaSize());
    laParent.resize(getlaSize());
    dlaParent.resize(getlaSize());
    rFactorParent.resize(getlaSize());
    sParent.resize(getlaSize());
    resParent.resize(getlaSize());
    gParent.resize(getgSize());
    gdParent.resize(getlaSize());
    zdParent.resize(getzSize());
    hParent.resize(getuSize());
    rParent.resize(getuSize());
    fParent.resize(getxSize());
    svParent.resize(getsvSize());
    jsvParent.resize(getsvSize());

    updatesvRef(svParent);
    updatejsvRef(jsvParent);
    updatehRef(hParent);
    updaterRef(rParent);
    updateMRef(MParent);
    updateTRef(TParent);
    updateLLMRef(LLMParent);
    G.resize() >> GParent;
    W.resize() >> WParent;
    b.resize() >> bParent;
    g.resize() >> gParent;

    updatezdRef(zdParent);

    Jh.resize(getuSize(),getzSize());

    if (xSize) cout<<"      xSize: " << xSize << endl;
    if (qSize) cout<<"      qSize: " << qSize << endl;
    if (uSize) cout<<"      uSize: " << uSize << endl;

    // single components
    if(INFO) cout << "  initialising ..." << endl;
    if(objects.size()>0 && INFO)  cout << "      " << objects.size() << " Objects" << endl;
    Object::init();
    for (vector<Object*>::iterator i = objects.begin(); i != objects.end(); ++i) (**i).init();

    if(links.size()>0 && INFO)    cout << "      " << links.size()   << " Links" << endl;
    for(vector<Link*>::iterator i = links.begin(); i != links.end(); ++i) {
      (**i).init();
      if(!(*i)->getHitSphereCheck()) {
	if((*i)->isSetValued()) {
	  if((*i)->isBilateral()) {
           linkSetValuedBilateral.push_back(*i);
           nHSLinksSetValuedBilateralFixed++;
          }
	  else {
	    linkSetValuedUnilateral.push_back(*i);
	    nHSLinksSetValuedUnilateralFixed++;
	  }
	}
	else {
	  nHSLinksSingleValuedFixed++;
	  linkSingleValued.push_back(*i);
	}
      }
    }

    if(linkSetValuedBilateral.size()>0 && INFO)  cout << "           " << linkSetValuedBilateral.size() <<" set valued bilateral"<<endl;
    if(linkSetValuedUnilateral.size()>0 && INFO) cout << "           " << linkSetValuedUnilateral.size()<<" set valued unilateral"<<endl;
    if(linkSingleValued.size()>0 && INFO)        cout << "           " << linkSingleValued.size()       <<" single valued"<<endl;


    if(EDI.size()>0 && INFO)    cout << "      " << EDI.size()   << " EDIs" << endl;
    for (vector<ExtraDynamicInterface*>::iterator i=EDI.begin(); i !=EDI.end(); ++i) (**i).init();

    // HitSphereLink
    if(HSLinks.size()>0 && INFO) cout << "  building " << HSLinks.size() << " HitSphereLinks between Objects" << endl;
    for(vector<HitSphereLink*>::iterator i = HSLinks.begin(); i != HSLinks.end(); ++i) (**i).init();

    for(vector<Link*>::iterator ic = links.begin(); ic != links.end(); ++ic) {
      if((*ic)->isSetValued()) (**ic).updategRef();
    }
    checkActiveConstraints();

    // solver specific settings
    if(INFO) cout << "  use solver \'" << getSolverInfo() << "\' for contact situations" << endl;
    if(solver == GaussSeidel) solve_ = &MultiBodySystem::solveGaussSeidel;
    else if(solver == LinearEquations) {
      solve_ = &MultiBodySystem::solveLinearEquations;
      cout << "WARNING: solveLL is only valid for bilateral constrained systems!" << endl;
      if (linkSetValuedUnilateral.size()) cout<<"          "<<linkSetValuedUnilateral.size()<<" unilateral set valued constraints."<<endl;
    }
    else if(solver == FixedPointSingle) solve_ = &MultiBodySystem::solveFixpointSingle;
    else if(solver == FixedPointTotal) solve_ = &MultiBodySystem::solveFixpointTotal;
    else if(solver == RootFinding)solve_ = &MultiBodySystem::solveRootFinding;
    else {
      cout << "ERROR (MultiBodySystem:init): unknown solver" << endl;
      throw 5;
    }

    if(INFO) cout << "  building plot lists, ";
    initPlotLists();
    if(INFO) cout << "writing parameter-files, ";
    plotParameterFiles();

    if(INFO) cout << "initialising plot-files ..." << endl;
    initPlotFiles();

    if(INFO) cout << "...... done initialising." << endl << endl;
  }

  void MultiBodySystem::initz(Vec& z) 
  {
    updatezRef(z);
    for(vector<Object*>::iterator i = objects.begin(); i != objects.end(); ++i) (**i).initz();
    for(vector<ExtraDynamicInterface*>::iterator iF = EDI.begin(); iF != EDI.end(); ++iF) (**iF).initz();
  }

  Vec MultiBodySystem::deltau(const Vec &zParent, double t, double dt) 
  {
    if(q()!=zParent()) updatezRef(zParent);

    // TODO update somewhere else
    updater(t); 
    updatedu(t,dt);
    return ud;
  }

  Vec MultiBodySystem::deltaq(const Vec &zParent, double t, double dt)
  {
    if(q()!=zParent()) updatezRef(zParent);
    updatedq(t,dt);
    return qd;
  }

  Vec MultiBodySystem::deltax(const Vec &zParent, double t, double dt) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }
    updatedx(t,dt);
    return xd;
  }

  bool MultiBodySystem::update(const Vec &zParent, double t) 
  {
    if(q()!=zParent()) updatezRef(zParent);
    updateKinematics(t);
    updateLinksStage1(t);
    bool ConstraintsChanged = activeConstraintsChanged;
    checkActiveConstraints();
    updateLinksStage2(t);
    updateT(t); 
    updateh(t); 
    updateM(t); 
    facLLM(); 
    updateW(t); 
    updateGb(t); 
    return ConstraintsChanged;
  }

  void MultiBodySystem::updateKinematics(double t)
  {
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (*i)->updateKinematics(t);
  }

  void MultiBodySystem::updateT(double t) {
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updateT(t);
  }

  void MultiBodySystem::facLLM() {
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).facLLM();
  }

  void MultiBodySystem::updateh(double t) 
  {
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updateh(t);
  }

  void MultiBodySystem::updateW(double t) {
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updateW(t);
  }

  void MultiBodySystem::updatew(double t) {
    vector<Link*>::iterator il;
    for(il = links.begin(); il!= links.end(); ++il)
      (**il).updatew(t);
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updatew(t);
  }

  void MultiBodySystem::updateGb(double t) 
  {
    G.init(0);
    b.init(0);

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updateGb(t);

    if(checkGSize) Gs.resize();
    else if(Gs.cols() != G.size()) {
      double facSizeGs = 1;
      if(G.size()>limitGSize && facSizeGs == 1) facSizeGs = double(countElements(G))/double(G.size()*G.size())*1.5;
      Gs.resize(G.size(),G.size(),int(G.size()*G.size()*facSizeGs));
    }
    Gs << G;
  }

  void MultiBodySystem::updateLinksStage1(double t)
  {
    if(!HSLinks.empty()) {
      linkSingleValued.erase(linkSingleValued.begin()+nHSLinksSingleValuedFixed,linkSingleValued.end());
      linkSetValuedUnilateral.erase(linkSetValuedUnilateral.begin()+nHSLinksSetValuedUnilateralFixed,linkSetValuedUnilateral.end());
      linkSetValuedBilateral.erase(linkSetValuedBilateral.begin()+nHSLinksSetValuedBilateralFixed,linkSetValuedBilateral.end());

      for(vector<HitSphereLink*>::iterator iHS = HSLinks.begin(); iHS != HSLinks.end(); ++iHS) (*iHS)->checkActive();
    }
    for(vector<ExtraDynamicInterface*>::iterator iF = EDI.begin(); iF != EDI.end(); ++iF) (*iF)->updateStage1(t);
    for(vector<Link*>::iterator iL = linkSingleValued.begin(); iL != linkSingleValued.end(); ++iL) (*iL)->updateStage1(t);
    for(vector<Link*>::iterator iL = linkSetValuedBilateral.begin();  iL != linkSetValuedBilateral.end(); ++iL)  (*iL)->updateStage1(t);
    for(vector<Link*>::iterator iL = linkSetValuedUnilateral.begin(); iL != linkSetValuedUnilateral.end(); ++iL) (*iL)->updateStage1(t);
  }

  void MultiBodySystem::updateLinksStage2(double t) 
  {
    for(vector<ExtraDynamicInterface*>::iterator iF = EDI.begin(); iF != EDI.end(); ++iF) 
      (*iF)->updateStage2(t);
    for(vector<Link*>::iterator iL = linkSingleValued.begin(); iL != linkSingleValued.end(); ++iL) 
      if((*iL)->isActive()) (*iL)->updateStage2(t); // only for active links! (TS, 09.05.2008)
    for(vector<Link*>::iterator iL = linkSetValuedBilateralActive.begin(); iL != linkSetValuedBilateralActive.end(); ++iL) 
      (*iL)->updateStage2(t);
    for(vector<Link*>::iterator iL = linkSetValuedUnilateralActive.begin(); iL != linkSetValuedUnilateralActive.end(); ++iL)
      (*iL)->updateStage2(t);
  }

  void MultiBodySystem::checkActiveConstraints() {
    if(activeConstraintsChanged) {
      laSize = 0;
      rFactorSize = 0;

      linkSetValuedBilateralActive.clear();
      linkSetValuedUnilateralActive.clear();
      vector<Link*>::iterator ic;

      for(ic = linkSetValuedBilateral.begin(); ic != linkSetValuedBilateral.end(); ++ic) {
	if((*ic)->isActive()) {
	  linkSetValuedBilateralActive.push_back(*ic);
	  (*ic)->setlaInd(laSize);
	  (*ic)->setrFactorInd(rFactorSize);
	  laSize += (*ic)->getlaSize();
	  rFactorSize += (*ic)->getrFactorSize();
	}
      }
      laIndBilateral = Index(0,laSize-1);
      for(ic = linkSetValuedUnilateral.begin(); ic != linkSetValuedUnilateral.end(); ++ic) {
	if((*ic)->isActive()) {
	  linkSetValuedUnilateralActive.push_back(*ic);
	  (*ic)->setlaInd(laSize);
	  (*ic)->setrFactorInd(rFactorSize);
	  laSize += (*ic)->getlaSize();
	  rFactorSize += (*ic)->getrFactorSize();
	}        
      }
      laIndUnilateral = Index(laIndBilateral.end()+1,laSize-1);

      W.resize() >> WParent(Index(0,getuSize()-1),Index(0,getlaSize()-1));
      G.resize() >> GParent(Index(0,getlaSize()-1));
      b.resize() >> bParent(Index(0,getlaSize()-1));

      la.resize() >> laParent(Index(0,getlaSize()-1));
      gd.resize() >> gdParent(Index(0,getlaSize()-1));
      s.resize() >> sParent(Index(0,getlaSize()-1));
      // TODO only Newton
      res.resize() >> resParent(Index(0,getlaSize()-1));
      rFactor.resize() >> rFactorParent(Index(0,getrFactorSize()-1));
      for(ic = linkSetValuedBilateralActive.begin();  ic != linkSetValuedBilateralActive.end(); ++ic)  (**ic).updateRef();
      for(ic = linkSetValuedUnilateralActive.begin(); ic != linkSetValuedUnilateralActive.end(); ++ic) (**ic).updateRef();

      activeConstraintsChanged = false;
      
      W.init(0);
      G.init(0);
    }
  }

  void MultiBodySystem::updatezRef(const Vec &zParent) 
  {
    q >> ( zParent(0,qSize-1));
    x >> ( zParent(qSize,qSize+xSize-1) );
    u >> ( zParent(qSize+xSize,qSize+xSize+uSize-1) );

    vector<Object*>::iterator i;     
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updatezRef();

    vector<Link*>::iterator ic;
    for(ic = links.begin(); ic != links.end(); ++ic) (**ic).updatexRef();

    vector<ExtraDynamicInterface*>::iterator iF;
    for(iF = EDI.begin(); iF != EDI.end(); ++iF) (**iF).updatexRef();
  }

  void MultiBodySystem::updater(double t)
  {
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updater(t);
  }

  int MultiBodySystem::solve(double dt) 
  {
    if(la.size()==0) return 0;
    if(useOldla) initla();
    else la.init(0);
    int iter;
    Vec laOld;
    laOld = la;
    iter = (this->*solve_)(dt); // solver evaluation (election in init())
    if(iter >= maxIter) {
      cout << endl;
      cout << "Iterations: " << iter << endl;
      cout << "\nError: no convergence."<< endl;
      if(stopIfNoConvergence) {
	if(dropContactInfo) dropContactMatrices();
	assert(iter < maxIter);
      }
      cout << "Anyway, continuing integration..."<< endl;
    }

    if(warnLevel>=1 && iter>highIter)
      cerr << endl << "WARNING (MultiBodySystem:solve): high number of iterations: " << iter << endl;

    if(useOldla) savela();

    return iter;
  }

  int MultiBodySystem::solveGaussSeidel(double dt) 
  {
    s = getgd() + getb()*dt; // new gd before \Lambda adaptation

    checkForTermination(dt);
    if(term) return 0 ;

    int iter;
    int checkTermLevel = 0;

    for(iter = 1; iter<=maxIter; iter++) {
      for(vector<Link*>::iterator ic = linkSetValuedBilateralActive.begin();  ic != linkSetValuedBilateralActive.end(); ++ic) 
	(**ic).solveGS(dt);
      for(vector<Link*>::iterator ic = linkSetValuedUnilateralActive.begin(); ic != linkSetValuedUnilateralActive.end(); ++ic) 
	(**ic).solveGS(dt); 
      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
	checkTermLevel++;
	checkForTermination(dt);
	if(term) break;
      }
    }
    return iter;
  }

  int MultiBodySystem::solveLinearEquations(double dt)
  {
    la = slvLL(G,-(getgd() + getb()*dt)); // new gd == 0 (only bilateral constraints)
    return 1;
  }

  int MultiBodySystem::solveFixpointSingle(double dt)
  {
    updaterFactors();
    s = getgd() + getb()*dt ;

    checkForTermination(dt);
    if(term) return 0;

    int iter, level = 0;
    int checkTermLevel = 0;

    for(iter = 1; iter<=maxIter; iter++) {

      if(level < decreaseLevels.size() && iter > decreaseLevels(level)) {
	level++;
	decreaserFactors();
	if(warnLevel>=2) cout << endl << "WARNING (MultiBodySystem:solveFixpointSingle): decreasing r-factors at iter = " << iter << endl;
      }
      for(vector<Link*>::iterator ic = linkSetValuedBilateralActive.begin();  ic != linkSetValuedBilateralActive.end(); ++ic)
	(*ic)->projectGS(dt);
      for(vector<Link*>::iterator ic = linkSetValuedUnilateralActive.begin(); ic != linkSetValuedUnilateralActive.end(); ++ic)
	(*ic)->projectGS(dt); 
      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
	checkTermLevel++;
	checkForTermination(dt);
	if(term) break;
      }
    }
    return iter;
  }

  int MultiBodySystem::solveFixpointTotal(double dt) 
{
    updaterFactors();
    Vec s0 = getgd() + getb()*dt ;
    s = s0;

    checkForTermination(dt);
    if(term) return 0 ;

    int iter, level = 0;

    for(iter = 1; iter<=maxIter; iter++) {
      double *a = getGs()();
      int *ia = getGs().Ip();
      int *ja = getGs().Jp();
      for(int i=0; i < G.size(); i++) {
	for(int j=ia[i]; j<ia[1+i]; j++) s(i) += a[j]*la(ja[j]);
      }

      if(level < decreaseLevels.size() && iter > decreaseLevels(level)) {
	level++;
	decreaserFactors();
	if(warnLevel>=2) cout << endl << "WARNING (MultiBodySystem:solveFixpointTotal): decreasing r-factors at iter = " << iter<<endl;
      }

      for(vector<Link*>::iterator ic = linkSetValuedBilateralActive.begin(); ic != linkSetValuedBilateralActive.end(); ++ic)
	(*ic)->projectJ(dt);
      for(vector<Link*>::iterator ic = linkSetValuedUnilateralActive.begin(); ic != linkSetValuedUnilateralActive.end(); ++ic)
	(*ic)->projectJ(dt); 

      s = s0;
      checkForTermination(dt);
      if(term) break;
    }
    return iter;
  }

  int MultiBodySystem::solveRootFinding(double dt)
  {
    updaterFactors();

    s = getgd() + getb()*dt;
    int iter;
    int checkTermLevel = 0;

    residualProj(dt); 
    double nrmf0 = nrm2(res);
    Vec res0 = res.copy();

    checkForTermination(dt);
    if(term) return 0;

    DiagMat I(la.size(),INIT,1);
    for(iter=1; iter<maxIter; iter++) {

      if(Jprox.size() != la.size()) Jprox.resize(la.size(),NONINIT);

      if(numJac) {
	double dx, xj;

	for(int j=0; j<la.size(); j++) {
	  xj = la(j);

	  dx = (epsroot * 0.5);
	  do dx += dx;
	  while (xj + dx == la(j));

	  la(j) += dx;
	  residualProj(dt);
	  la(j) = xj;
	  Jprox.col(j) = (res-res0)/dx;
	}
      } 
      else residualProjJac(dt);
      Vec dx;
      if(linAlg == LUDecomposition) dx >> slvLU(Jprox,res0);
      else if(linAlg == LevenbergMarquardt) {
	SymMat J = SymMat(JTJ(Jprox) + lmParm*I);
	dx >> slvLL(J,trans(Jprox)*res0);
      }
      else if(linAlg == PseudoInverse) dx >> slvLS(Jprox,res0);
      else throw 5;

      double alpha = 1;       

      Vec La_old = la.copy();

      double nrmf = 1;
      for(int k=0; k<maxDampingSteps; k++) {
	la = La_old - alpha*dx;
	residualProj(dt);
	nrmf = nrm2(res);
	if(nrmf < nrmf0) break;

	alpha = 0.5*alpha;  
      }
      nrmf0 = nrmf;
      res0 = res;

      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
	checkTermLevel++;
	checkForTermination(dt);
	if(term) break;
      }
    }
    return iter;
  }

  void MultiBodySystem::residualProj(double dt) 
  {
    for(vector<Link*>::iterator ic = linkSetValuedBilateralActive.begin();  ic != linkSetValuedBilateralActive.end(); ++ic) 
      (**ic).residualProj(dt);
    for(vector<Link*>::iterator ic = linkSetValuedUnilateralActive.begin(); ic != linkSetValuedUnilateralActive.end(); ++ic) 
      (**ic).residualProj(dt); 
  }

  void MultiBodySystem::residualProjJac(double dt) {
    for(vector<Link*>::iterator ic = linkSetValuedBilateralActive.begin(); ic != linkSetValuedBilateralActive.end(); ++ic)
      (**ic).residualProjJac(dt);
    for(vector<Link*>::iterator ic = linkSetValuedUnilateralActive.begin(); ic != linkSetValuedUnilateralActive.end(); ++ic)
      (**ic).residualProjJac(dt); 
  }

  void MultiBodySystem::checkForTermination(double dt) 
  {
    term = true;
    for(vector<Link*>::iterator ic = linkSetValuedBilateralActive.begin(); ic != linkSetValuedBilateralActive.end(); ++ic) {
      (**ic).checkForTermination(dt);
      if(term == false) return;
    }
    for(vector<Link*>::iterator ic = linkSetValuedUnilateralActive.begin(); ic != linkSetValuedUnilateralActive.end(); ++ic) {
      (**ic).checkForTermination(dt);
      if(term == false) return;
    }
  }

  void MultiBodySystem::dropContactMatrices() 
  {
    cout << "dropping contact matrices to file <dump_matrices.asc>" << endl;
    ofstream contactDrop("dump_matrices.asc");   

    contactDrop << "constraint functions g" << endl << trans(g) << endl << endl;
    contactDrop << endl;
    contactDrop << "mass matrix M" << endl << M << endl << endl;
    contactDrop << "generalized force directions W" << endl << W << endl << endl;
    contactDrop << "Delassus matrix G" << endl << G << endl << endl;
    contactDrop << endl;
    contactDrop << "constraint velocities gp" << endl << trans(gd) << endl << endl;
    contactDrop << "non-holonomic part in gp; b" << endl << trans(b) << endl << endl;
    contactDrop << "Lagrange multipliers la" << endl << trans(la) << endl << endl;
    contactDrop.close();
  }

  void MultiBodySystem::updateM(double t) 
  {
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) {
      (**i).updateM(t);
    }
  }

  void MultiBodySystem::savela() {
    vector<Link*>::iterator ic;
    for(ic = linkSetValuedBilateralActive.begin();  ic != linkSetValuedBilateralActive.end(); ++ic) (**ic).savela();
    for(ic = linkSetValuedUnilateralActive.begin(); ic != linkSetValuedUnilateralActive.end(); ++ic) (**ic).savela();

  }

  void MultiBodySystem::initla() {
    vector<Link*>::iterator ic;
    for(ic = linkSetValuedBilateralActive.begin();  ic != linkSetValuedBilateralActive.end(); ++ic) (**ic).initla();
    for(ic = linkSetValuedUnilateralActive.begin(); ic != linkSetValuedUnilateralActive.end(); ++ic) (**ic).initla();
  }

  void MultiBodySystem::setScaleTolQ(double scaleTolQ) {
    vector<Link*>::iterator i;
    for(i = links.begin(); i!= links.end(); ++i)
      (**i).setScaleTolQ(scaleTolQ);
  }

  void MultiBodySystem::setScaleTolp(double scaleTolp) {
    vector<Link*>::iterator i;
    for(i = links.begin(); i!= links.end(); ++i)
      (**i).setScaleTolp(scaleTolp);
  }

  void MultiBodySystem::setgdTol(double tol) {
    vector<Link*>::iterator i;
    for(i = links.begin(); i!= links.end(); ++i)
      (**i).setgdTol(tol);
  }

  void MultiBodySystem::setlaTol(double tol) {
    vector<Link*>::iterator i;
    for(i = links.begin(); i!= links.end(); ++i)
      (**i).setlaTol(tol);
  }

  void MultiBodySystem::setrMax(double rMax) {
    vector<Link*>::iterator i;
    for(i = links.begin(); i!= links.end(); ++i)
      (**i).setrMax(rMax);
  }

  void MultiBodySystem::decreaserFactors() 
  {
    for(vector<Link*>::iterator i = linkSetValuedBilateralActive.begin(); i != linkSetValuedBilateralActive.end(); ++i)
      (*i)->decreaserFactors();
    for(vector<Link*>::iterator i = linkSetValuedUnilateralActive.begin(); i != linkSetValuedUnilateralActive.end(); ++i)
      (*i)->decreaserFactors();
  }

  string MultiBodySystem::getSolverInfo() 
  {
    stringstream info;

    // Solver-Name
    if(solver == GaussSeidel) info << "GaussSeidel";
    else if(solver == LinearEquations) info << "LinearEquations";
    else if(solver == FixedPointSingle) info << "FixedPointSingle";
    else if(solver == FixedPointTotal) info << "FixedPointTotal";
    else if(solver == RootFinding) info << "RootFinding";

    // Gauss-Seidel & solveLL do not depend on the following ...
    if(solver!=GaussSeidel && solver!=LinearEquations) {
      info << "(";

      // r-Factor strategy
      if(strategy==global) info << "global";
      else if(strategy==local) info << "local";

      // linear algebra for RootFinding only
      if(solver == RootFinding) {
	info << ",";
	if(linAlg==LUDecomposition) info << "LU";
	else if(linAlg==LevenbergMarquardt) info << "LM";
	else if(linAlg==PseudoInverse) info << "PI";
      }
      info << ")";
    }
    return info.str();
  }

  void MultiBodySystem::initPlotLists() {
    // plot-Listen aufbauen
    //  vector<Object*>::iterator i;
    for(vector<Object*>::iterator i = objects.begin(); i != objects.end(); ++i) {
      if((**i).getPlotLevel()>0) objects2plot.push_back((*i));
      for(vector<Contour*>::iterator i4 = (**i).contour.begin(); i4!= (**i).contour.end(); ++i4)
	if((**i4).getPlotLevel()>0) contours2plot.push_back((*i4));
      for(vector<Port*>::iterator i5 = (**i).port.begin(); i5!= (**i).port.end(); ++i5)
	// jetzt in Port::init()      (**i5).setFullName((**i).getFullName()+ "."+ (**i5).getFullName()); // etl. problem. bei Namens-Aufloesungen durch Vergleiche: pruefen
	if((**i5).getPlotLevel()>0) ports2plot.push_back((*i5));
    }

    for(vector<Link*>::iterator i2 = links.begin(); i2!= links.end(); ++i2)
      if((**i2).getPlotLevel()>0) links2plot.push_back((*i2));

    for(vector<Contour*>::iterator i4 = contour.begin(); i4!= contour.end(); ++i4)
      if((**i4).getPlotLevel()>0) contours2plot.push_back((*i4));
    for(vector<Port*>::iterator i5 = port.begin(); i5!= port.end(); ++i5)
      if((**i5).getPlotLevel()>0) ports2plot.push_back((*i5));

    for(vector<ExtraDynamicInterface*>::iterator i3 = EDI.begin(); i3!= EDI.end(); ++i3)
      if((**i3).getPlotLevel()>0) EDIs2plot.push_back((*i3));
  }

  void MultiBodySystem::initPlotFiles() {
    Object::initPlotFiles();
    // Energieterme
    if(plotLevel>=3) {
      plotfile <<"# " << plotNr++ << ": T" << endl;
      plotfile <<"# " << plotNr++ << ": V" << endl;
      plotfile <<"# " << plotNr++ << ": E" << endl;
    }   

    // initialization of plot-active elements
    for(vector<Object*>::iterator                i = objects2plot.begin();  i != objects2plot.end(); ++i)  (**i).initPlotFiles();
    for(vector<Link*>::iterator                  i = links2plot.begin();    i != links2plot.end(); ++i)    (**i).initPlotFiles();
    for(vector<Contour*>::iterator               i = contours2plot.begin(); i != contours2plot.end(); ++i) (**i).initPlotFiles();
    for(vector<Port*>::iterator                  i = ports2plot.begin();    i != ports2plot.end(); ++i)    (**i).initPlotFiles();
    for(vector<ExtraDynamicInterface*>::iterator i = EDIs2plot.begin();     i != EDIs2plot.end(); ++i)     (**i).initPlotFiles();
  }

  void MultiBodySystem::closePlotFiles() {
    Object::closePlotFiles();
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) {
      (**i).closePlotFiles();
      vector<Contour*>::iterator i4;
      for(i4 = (**i).contour.begin(); i4!= (**i).contour.end(); ++i4)
	(**i4).closePlotFiles();
    }
    vector<Link*>::iterator i2;
    for(i2 = links.begin(); i2!= links.end(); ++i2)
      (**i2).closePlotFiles();
    vector<ExtraDynamicInterface*>::iterator i3;
    for(i3 = EDI.begin(); i3!= EDI.end(); ++i3)
      (**i3).closePlotFiles();
  }

  void MultiBodySystem::plotParameters() {
    parafile << "#MultibodySystem\n" << fullName << endl;
    parafile << "#solver\n" << getSolverInfo() << endl;

    // all Objects of MultibodySystem
    if(objects.size()>0) {
      parafile << "\n#Objects" << endl;
      for(vector<Object*>::iterator i = objects2plot.begin();  i != objects2plot.end();  ++i)
	parafile << "  " << (**i).getName() << endl;
    }
    // all Ports to environment
    if(port.size()>0) {
      parafile << "\n#environmental ports" << endl;
      for(vector<Port*>::iterator i = port.begin();  i != port.end();  ++i) {
	Vec WrOPtemp = (**i).getWrOP();
	parafile << "  KrSP: (port:  name= "<<(**i).getName()<<",  ID= "<<(**i).getID()<<") = (" << WrOPtemp(0) <<","<< WrOPtemp(1) <<","<< WrOPtemp(2) << ")" << endl;
      } 
    }
    // all Contours of environment
    if(contour.size()>0) {
      parafile << "\n#environmental contours" << endl;
      for(vector<Contour*>::iterator i = contour.begin();  i != contour.end();  ++i)
	parafile << "  " << (**i).getName() << endl;
    }
  }

  void MultiBodySystem::plotParameterFiles() {
    Object::plotParameterFiles();
    for(vector<Object*>::iterator  i = objects2plot.begin();  i != objects2plot.end();  ++i) (**i).plotParameterFiles();
    for(vector<Link*>::iterator    i = links2plot.begin();    i != links2plot.end();    ++i) (**i).plotParameterFiles();
    for(vector<Contour*>::iterator i = contours2plot.begin(); i != contours2plot.end(); ++i) (**i).plotParameterFiles();
    for(vector<Port*>::iterator    i = ports2plot.begin();    i != ports2plot.end();    ++i) (**i).plotParameterFiles();
  }

  void MultiBodySystem::plot(double t, double dt) {

    for(vector<Object*>::iterator                i = objects2plot.begin();  i != objects2plot.end(); ++i)  (**i).plot(t,dt);
    for(vector<Link*>::iterator                  i = links2plot.begin();    i != links2plot.end(); ++i)    (**i).plot(t,dt);
    for(vector<Contour*>::iterator               i = contours2plot.begin(); i != contours2plot.end(); ++i) (**i).plot(t,dt);
    for(vector<Port*>::iterator                  i = ports2plot.begin();    i != ports2plot.end(); ++i)    (**i).plot(t,dt);
    for(vector<ExtraDynamicInterface*>::iterator i = EDIs2plot.begin();     i != EDIs2plot.end(); ++i)     (**i).plot(t,dt);

    Object::plot(t,dt);

    /* Gesamtsystem Energien */
    if(plotLevel>=3) {
      double Ttemp = this->computeKineticEnergy();
      double Vtemp = this->computePotentialEnergy();
      plotfile<<" "<< Ttemp;
      plotfile<<" "<< Vtemp;
      plotfile<<" "<< Ttemp + Vtemp;
    }
  }

  void MultiBodySystem::computeConstraintForces(double t) {
    la = slvLL(G, -b);
  }

  void MultiBodySystem::projectViolatedConstraints(double t) 
  { 
    if(laSize) {
      Vec nu(uSize);
      int gASize = 0;
      unsigned int sizeBilateral  = linkSetValuedBilateralActive.size();
      unsigned int sizeUnilateral = linkSetValuedUnilateralActive.size();
      for(unsigned int i = 0; i<sizeBilateral; i++)  gASize += linkSetValuedBilateralActive[i]->getgSize();
      for(unsigned int i = 0; i<sizeUnilateral; i++)   gASize += linkSetValuedUnilateralActive[i]->getgSize();

      SymMat Gv(gASize,NONINIT);
      Mat Wv(W.rows(),gASize,NONINIT);
      Vec gv(gASize,NONINIT);
      int gAIndi = 0;
      vector<Link*>::iterator it,jt;
      if (sizeBilateral) it = linkSetValuedBilateralActive.begin();
      else it = linkSetValuedUnilateralActive.begin();
      for(unsigned int i = 0; i<(sizeBilateral+sizeUnilateral); i++) {
	Index I1 = Index((*it)->getlaInd(),(*it)->getlaInd()+(*it)->getgSize()-1);
	Index Iv = Index(gAIndi,gAIndi+(*it)->getgSize()-1);
	Wv(Index(0,Wv.rows()-1),Iv) = W(Index(0,W.rows()-1),I1);
	gv(Iv) = g((*it)->getgIndex());
	Gv(Iv) = G(I1);
	int gAIndj = 0;
	if (sizeBilateral) jt = linkSetValuedBilateralActive.begin();
	else jt = linkSetValuedUnilateralActive.begin();
	for(unsigned int j = 0; j<i; j++) {
	  Index Jv = Index(gAIndj,gAIndj+(*jt)->getgSize()-1);
	  Index J1 = Index((*jt)->getlaInd(),(*jt)->getlaInd()+(*jt)->getgSize()-1);
	  Gv(Jv,Iv)= G(J1,I1);
	  gAIndj  += (*jt)->getgSize();
	  if (j==sizeBilateral-1) jt=linkSetValuedUnilateralActive.begin(); 
	  else jt++;
	}
	gAIndi+=(*it)->getgSize();
	if (i==sizeBilateral-1) it = linkSetValuedUnilateralActive.begin();
	else it++;
      }
      while(nrmInf(gv) >= 1e-8) {
	Vec mu = slvLL(Gv, -gv+trans(Wv)*nu);
	Vec dnu = slvLLFac(LLM,Wv*mu- M*nu);
	nu += dnu;
	q += T*dnu;
	updateKinematics(t);
	updateLinksStage1(t);
	int gAIndi = 0;
	if (sizeBilateral) it = linkSetValuedBilateralActive.begin();
	else it = linkSetValuedUnilateralActive.begin();
	for(unsigned int i = 0; i<(sizeBilateral+sizeUnilateral); i++) {
	  Index I1 = Index((*it)->getlaInd(),(*it)->getlaInd()+(*it)->getgSize()-1);
	  Index Iv = Index(gAIndi,gAIndi+(*it)->getgSize()-1);
	  gv(Iv) = g((*it)->getgIndex());
	  gAIndi+= (*it)->getgSize();
	  if (i==sizeBilateral-1) it = linkSetValuedUnilateralActive.begin();
	  else it++;
	}
      }
    }
  }

  void MultiBodySystem::preInteg(MultiBodySystem *parent){
    if(preIntegrator){
      setProjectDirectory(name+".preInteg");
      setGrav(parent->getGrav()); //TODO gravitation has to be set for preintegration
      if(INFO) cout << "Initialisation of " << name << " for Preintegration..." << endl;
      init();  
      if(INFO) cout << "Preintegration..." << endl;
      preIntegrator->integrate(*this);
      closePlotFiles();
      writez();
      delete preIntegrator;
      preIntegrator=NULL; 
      if(INFO) cout << "Finished." << endl;
    }  
  }

  void MultiBodySystem::writez(){
    for(unsigned int i=0; i<objects.size(); i++) {
      objects[i]->writeq();
      objects[i]->writeu();
      objects[i]->writex();
    }
    for(unsigned int i=0; i<EDI.size(); i++) EDI[i]->writex();
  }

  void MultiBodySystem::readz0(){
    for(unsigned int i=0; i<objects.size(); i++) {
      objects[i]->readq0();
      objects[i]->readu0();
      objects[i]->readx0();
    }
    for(unsigned int i=0; i<EDI.size(); i++) EDI[i]->readx0();
  }

  void MultiBodySystem::plot(const Vec& zParent, double t, double dt) {   
    if(q()!=zParent()) updatezRef(zParent);
    if(qd()!=zdParent()) updatezdRef(zdParent);
    updateKinematics(t);
    updateLinksStage1(t);
    if(t==0.) checkActiveConstraints(); // gaps depend on t and q
    updateLinksStage2(t);
    updateh(t); // TODO necessary for ODE-Integration with high plotlevel
    updateM(t); 
    //updateG(t); 
    //computeConstraintForces(t); 
    //updater(t); 
    //updatezd(t);

    plot(t,dt);
  }

  void MultiBodySystem::plotDAE(const Vec& YParent, double t, int DAEIndex) { 
    int zSize = getzSize();
    int laBilateralSize = getlaBilateralSize();
    Vec zParent;
    zParent >> YParent(0,zSize-1); 
    if (DAEIndex>1) la(laIndBilateral) = YParent(zSize,zSize+laBilateralSize-1);
    if(q()!=zParent()) updatezRef(zParent);
    if(qd()!=zdParent()) updatezdRef(zdParent);
    updateKinematics(t);
    updateLinksStage1(t);
    if(t==0.) checkActiveConstraints(); // gaps depend on t and q
    updateLinksStage2(t);
    updateh(t); // TODO necessary for ODE-Integration with high plotlevel
    updateM(t); 
    facLLM(); 
    if (DAEIndex==1) {
       updateW(t);
       updateGb(t);
       updatew(t);
       computeConstraintForces(t);
       //updater(t);
    }
    else la(laIndBilateral) = YParent(zSize,zSize+laBilateralSize-1);
    int IndexlaUnilaterlal = zSize+laBilateralSize;
    for(vector<Link*>::iterator ic = linkSetValuedUnilateral.begin(); ic != linkSetValuedUnilateral.end(); ++ic) {
      int lmSize = (*ic)->getSizeConstraints();
      (*ic)->setLagrangeMultiplier(YParent(IndexlaUnilaterlal,IndexlaUnilaterlal+lmSize-1));
      IndexlaUnilaterlal += lmSize;
    }


 
    updatezd(t);
    plot(t,1);

  }

  void MultiBodySystem::updateJh(double t) 
  {
    Jh.init(0.0);
    for (vector<Object*>::iterator i = objects.begin(); i != objects.end(); ++i) (**i).updateJh(t);
  }

  Vec MultiBodySystem::zdot(const Vec &zParent, double t) {
    if(q()!=zParent()) updatezRef(zParent);
    updateKinematics(t);
    updateLinksStage1(t);
    updateLinksStage2(t);
    updateT(t); 
    updateh(t); 
    updateM(t); 
    facLLM(); 
    if(linkSetValuedBilateral.size() || linkSetValuedUnilateral.size()) {
      updateW(t); 
      updateGb(t); 
      updatew(t);
      computeConstraintForces(t); 
      updater(t); 
    }
    updatezd(t);

    return zdParent;
  }

  void MultiBodySystem::zdot(const Vec &zParent, Vec &zdParent, double t) {
    if(qd()!=zdParent()) updatezdRef(zdParent);
    zdot(zParent,t);
  }

  void MultiBodySystem::root_DAE(const Vec &YParent, Vec &rt, double t) {
    int zSize = getzSize();
    int laBilateralSize = getlaBilateralSize();
    if(q()!=YParent()) {
      updatezRef(YParent(0,zSize-1));
    }
    if(sv()!=rt()) updatesvRef(rt);
    updateKinematics(t);
    updateLinksStage1(t);
    la(laIndBilateral) = YParent(zSize,zSize+laBilateralSize-1);
    int IndexlaUnilaterlal = zSize+laBilateralSize;
    for(vector<Link*>::iterator ic = linkSetValuedUnilateral.begin(); ic != linkSetValuedUnilateral.end(); ++ic) {
      int lmSize = (*ic)->getSizeConstraints();
      (*ic)->setLagrangeMultiplier(YParent(IndexlaUnilaterlal,IndexlaUnilaterlal+lmSize-1));
      IndexlaUnilaterlal += lmSize;
    }
    updateLinksStage2(t);
    updateStopVector(t);
  }

  void MultiBodySystem::saveUnilaterLinkStatus() {
    svSize =0;
    constraintSize =0;
    for(vector<Link*>::iterator ic = linkSetValuedUnilateral.begin(); ic != linkSetValuedUnilateral.end(); ++ic) {
      (*ic)->saveStatus();
      svSize += (*ic)->getsvSize();
      constraintSize += (*ic)->getSizeConstraints();
    }
    sv.resize(svSize);
  }


  void MultiBodySystem::F_DAE(const Vec &YParent, Vec &F, double t, int DAEIndex)   // F=[zdot; g(dot)] z.B. fuer Radau5
  {
    int zSize = getzSize();
    int laBilateralSize = getlaBilateralSize();  
    int IndexlaUnilaterlal;
    if(qd()!=F()) {
      zdParent >> F(0,zSize-1); 
      updatezdRef(zdParent);
    }
    if(q()!=YParent()) {
      updatezRef(YParent(0,zSize-1));
    }
    updateKinematics(t);
    updateLinksStage1(t);
    checkActiveConstraints(); // nicht notwendig bei reinen zweiseitigen DAE; aber erforderlich bei einseitigen Bindungen und
    updateLinksStage2(t);     // und Schaltpunktsuche: zur Schaltpunktsuche integriert Integrator auch ueber g=0 hinaus
    updateT(t); 	      // dann wird contact auf aktiv gesetzt und in nachfolgenden Routinen z.B. update W werden
    updateh(t); 	      // daten aus updatestage2 benoetig; updatestage2 wird aber nur durchgeführt, wenn link
    updateM(t); 	      // mittels checkActiveConstraints in Cvector linkSetValuedUnilateralActive eingetragen wurde
    facLLM(); 
    if(linkSetValuedBilateral.size() || linkSetValuedUnilateral.size()) {
      updateW(t); 
      updateGb(t); 
      if (DAEIndex==1)  {
        computeConstraintForces(t);
        updatew(t);
      }
      else la(laIndBilateral) = YParent(zSize,zSize+laBilateralSize-1);
    IndexlaUnilaterlal = zSize+laBilateralSize;
    for(vector<Link*>::iterator ic = linkSetValuedUnilateral.begin(); ic != linkSetValuedUnilateral.end(); ++ic) {
      int lmSize = (*ic)->getSizeConstraints();
      (*ic)->setLagrangeMultiplier(YParent(IndexlaUnilaterlal,IndexlaUnilaterlal+lmSize-1));
      IndexlaUnilaterlal += lmSize;
    }

      updater(t); 
    }
    updatezd(t);
    if (DAEIndex == 2)  {
      F(zSize,zSize+laBilateralSize-1) = gd(laIndBilateral);
      IndexlaUnilaterlal = zSize+laBilateralSize;
       for(vector<Link*>::iterator ic = linkSetValuedUnilateral.begin(); ic != linkSetValuedUnilateral.end(); ++ic) {
         int lmSize = (*ic)->getSizeConstraints();
         Vec tmpF;
         tmpF >> F(IndexlaUnilaterlal,IndexlaUnilaterlal+lmSize-1);
         (*ic)->getConstraints(tmpF,2);
         IndexlaUnilaterlal += lmSize;
       }
    }
    if (DAEIndex == 3)  F(zSize,zSize+laBilateralSize-1) = g(gIndBilateral); //TODO Unilateral
    if (DAEIndex == 21) { // Gear Gupta Leimkuhler Formulation (GGL)        // TODO Unilateral
      F(zSize,zSize+laBilateralSize-1) = g(gIndBilateral);
      F(zSize+laBilateralSize,zSize+laBilateralSize+laBilateralSize-1) = gd(laIndBilateral);
      qd += W(Index(0,uSize-1),laIndBilateral) * YParent(zSize+laBilateralSize, zSize+laBilateralSize+laBilateralSize-1);
    }
  }

  // Jacobian dF/dY (F=[f(z,t); g(dot)] Y=[z;la]
  void MultiBodySystem::JacF_DAE(double t, const Vec &YParent, Mat &Jac, int DAEIndex) 
  {
    int YSize = YParent.size();
    int zSize = getzSize();
    int laBilateralSize = getlaBilateralSize();
    double deltaEPS = epsroot;
    double delta;
    static Vec F(YSize);
    static Vec Y(YSize);
    Vec F0(YSize);
    Vec Fdelta(YSize);
    Vec Ydelta(YSize,NONINIT);
    Y=YParent;
    F_DAE(Y, F, t, DAEIndex);
    F0 = F;
    for(int i=0; i<zSize; i++) {
      Ydelta = YParent;
      delta = fabs(Ydelta(i));
      if (delta>0.001) delta = delta*deltaEPS;
      else delta = deltaEPS*0.001;
      Ydelta(i)+= delta;
      Y = Ydelta;
      F_DAE(Y, F, t, DAEIndex);
      Fdelta =  F;
      Jac.col(i) = (Fdelta-F0)/delta;
    }
   Jac(xSize+qSize,zSize,zSize-1,zSize+laBilateralSize-1) = slvLLFac(LLM,W(Index(0,uSize-1),laIndBilateral));
   if (DAEIndex == 21) Jac(0,zSize+laBilateralSize,qSize-1,zSize+laBilateralSize+laBilateralSize-1) = W(Index(0,uSize-1),laIndBilateral);
  }

  void MultiBodySystem::getsv(const Vec& zParent, Vec& svExt, double t) 
  {
    if(sv()!=svExt()) updatesvRef(svExt);
    if(q()!=zParent()) updatezRef(zParent);
    if(qd()!=zdParent()) updatezdRef(zdParent);
    updateKinematics(t);
    updateLinksStage1(t);
    updateLinksStage2(t);
    updateh(t);
    if(linkSetValuedBilateral.size() || linkSetValuedUnilateral.size()) {
      updateW(t); 
      updateGb(t); 
      updatew(t);
      computeConstraintForces(t);
    }
    updateStopVector(t);
  }

  void MultiBodySystem::initDataInterfaceBase() 
  {
    vector<Link*>::iterator il1;
    for(il1 = links.begin(); il1 != links.end(); ++il1) (*il1)->initDataInterfaceBase(this);
    vector<Object*>::iterator io1;
    for(io1 = objects.begin(); io1 != objects.end(); ++io1) (*io1)->initDataInterfaceBase(this);
    vector<ExtraDynamicInterface*>::iterator ie1;
    for(ie1 = EDI.begin(); ie1 != EDI.end(); ++ie1) (*ie1)->initDataInterfaceBase(this); 
  }

  void MultiBodySystem::updatezdRef(const Vec &zdParent) {
    qd >> ( zdParent(0,qSize-1) );
    xd >> ( zdParent(qSize,qSize+xSize-1) );
    ud >> ( zdParent(qSize+xSize,qSize+xSize+uSize-1) );

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updatezdRef();

    vector<Link*>::iterator ic;
    for(ic = links.begin(); ic != links.end(); ++ic) (**ic).updatexdRef();

    vector<ExtraDynamicInterface*>::iterator iF;
    for(iF = EDI.begin(); iF != EDI.end(); ++iF) (**iF).updatexdRef();
  }

  void MultiBodySystem::updateqRef(const Vec &qParent) 
  {

    q >> qParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updateqRef();
  }

  void MultiBodySystem::updateqdRef(const Vec &qdExt) 
  {

    qd >> qdExt;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updateqdRef();
  }

  void MultiBodySystem::updateuRef(const Vec &uParent) {

    u >> uParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updateuRef();
  }

  void MultiBodySystem::updatexRef(const Vec &xParent) {

    x >> xParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updatexRef();

    vector<Link*>::iterator i2;
    for(i2 = links.begin(); i2 != links.end(); ++i2) (**i2).updatexRef();

    vector<ExtraDynamicInterface*>::iterator iF;
    for(iF = EDI.begin(); iF != EDI.end(); ++iF) (**iF).updatexRef();
  }

  void MultiBodySystem::updatehRef(const Vec &hParent) {

    h >> hParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updatehRef();
  }  

  void MultiBodySystem::updaterRef(const Vec &hParent) {

    r >> rParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updaterRef();
  }

  void MultiBodySystem::updatefRef(const Vec &fParent) {

    f >> fParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updatefRef();
  }

  void MultiBodySystem::updateTRef(const Mat &TParent) {

    T >> TParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updateTRef();
  }

  void MultiBodySystem::updateMRef(const SymMat &MParent) {

    M >> MParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updateMRef();
  }

  void MultiBodySystem::updateLLMRef(const SymMat &LLMParent) {

    LLM >> LLMParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updateLLMRef();
  }

  void MultiBodySystem::updatesvRef(const Vec &svExt) {

    sv >> svExt;

    vector<Link*>::iterator ic;
    for(ic = links.begin(); ic != links.end(); ++ic) (**ic).updatesvRef();
  }

  void MultiBodySystem::updatejsvRef(const Vector<int> &jsvExt) {

    jsv >> jsvExt;

    vector<Link*>::iterator ic;
    for(ic = links.begin(); ic != links.end(); ++ic) (**ic).updatejsvRef();
  }

  void MultiBodySystem::updaterFactors()
  {
    if(strategy == global) {
      double rFac;
      if(G.size() == 1) rFac = 1./G(0,0);
      else {
	Vec eta = eigvalSel(G,1,G.size());
	double etaMax = eta(G.size()-1);
	double etaMin = eta(0);
	int i=1;
	while(abs(etaMin) < 1e-8 && i<G.size()) etaMin = eta(i++);
	rFac = 2./(etaMax + etaMin);
      }
      rFactor.init(rFac);

    }
    else if(strategy == local) {
      for(vector<Link*>::iterator i = linkSetValuedBilateralActive.begin(); i != linkSetValuedBilateralActive.end(); ++i)
	(**i).updaterFactors();
      for(vector<Link*>::iterator i = linkSetValuedUnilateralActive.begin(); i != linkSetValuedUnilateralActive.end(); ++i)
	(**i).updaterFactors(); 
    } 
    else {
      cout << "ERROR (MultiBodySystem:updaterFactors): Unknown strategy" << endl;
      throw 5;
    }
  }

  void MultiBodySystem::updatezd(double t)
  {

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updatezd(t);

    vector<Link*>::iterator il;
    for(il = links.begin(); il!= links.end(); ++il) (**il).updatexd(t);

    vector<ExtraDynamicInterface*>::iterator iF;
    for(iF = EDI.begin(); iF!= EDI.end(); ++iF) (**iF).updatexd(t);
  }

  void MultiBodySystem::updatedx(double t, double dt) {
    // TODO set limits to list
    vector<Object*>::iterator io;
    for(io = objects.begin(); io != objects.end(); ++io) 
      (**io).updatedx(t,dt);

    // TODO set limits to list
    vector<Link*>::iterator il;
    for(il = links.begin(); il!= links.end(); ++il)
      (**il).updatedx(t,dt);

    vector<ExtraDynamicInterface*>::iterator iF;
    for(iF = EDI.begin(); iF!= EDI.end(); ++iF)
      (**iF).updatedx(t,dt);
  }

  void MultiBodySystem::updatedu(double t, double dt) 
  {
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updatedu(t,dt);
  }

  void MultiBodySystem::updatedq(double t, double dt)
  {
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) (**i).updatedq(t,dt);
  }

  void MultiBodySystem::updateStopVector(double t) 
  {
    vector<Link*>::iterator ic;
    for(ic = links.begin(); ic != links.end(); ++ic) (*ic)->updateStopVector(t); 
  }

  void MultiBodySystem::setDirectory() 
  {
    int i;
    string projectDirectory;

    if(directoryName == name) { // numered directories
      for(i=0; i<=99; i++) {
	stringstream number;
	number << "." << setw(2) << setfill('0') << i;
	projectDirectory = directoryName + number.str();
	int ret = mkdir(projectDirectory.c_str(),0777);
	if(ret == 0) break;
      }
      if(INFO) cout << "  make directory \'" << projectDirectory << "\' for output processing" << endl;
    }
    else { // always the same directory
      projectDirectory = string(directoryName);

      int ret = mkdir(projectDirectory.c_str(),0777);
      if(ret == 0) {
	if(INFO) cout << "  make directory \'" << projectDirectory << "\' for output processing" << endl;
      }
      else {
	if(INFO) cout << "  use existing directory \'" << projectDirectory << "\' for output processing" << endl;
      }
    }

    dirName = projectDirectory+"/";

    if(preIntegrator) {
      string preDir="PREINTEG";
      int ret=mkdir(preDir.c_str(),0777);
      if(ret==0) {
      if(INFO) cout << "Make directory " << preDir << " for Preintegration results." << endl;
      }
      else {
	if(INFO) cout << "Use existing directory " << preDir << " for Preintegration results." << endl;
      }
    }
  }

  void MultiBodySystem::assembleSystem(double t0) {
    cout<< "Start Assembling System ..."<<endl;
    int gSizeInitial = 0;
    int q0FixedSize = 0;
    for(vector<Link*>::iterator il = links.begin(); il != links.end(); ++il)  {
      if ((*il)->isActiveforAssembling() && (*il)->getgSize()) {
	linksInitialActive.push_back(*il);
	gSizeInitial += (*il)->getgSize();
      }
    }
    Vector<int> q0FixedIndexTmp(qSize);
    for (vector<Object*>::iterator io = objects.begin(); io != objects.end(); ++io) {
      Vector<int> q0fix = (*io)->getq0fixed();
      int qIndStart = (*io)->getqInd();
      for (int i=0; i < q0fix.size(); i++) {
	q0FixedIndexTmp(q0FixedSize) = qIndStart + q0fix(i);
	q0FixedSize++;
      }
    }
    Vector<int> q0FixedIndex(q0FixedSize);
    q0FixedIndex = q0FixedIndexTmp(0,q0FixedSize-1);
    qVarIndex.resize(qSize-q0FixedSize);    
    int jj=0;
    for(int i=0; i<qSize; i++) {
      bool isFixed = false;
      for(int j=0; j<q0FixedSize; j++) {
	if (q0FixedIndex(j)==i) isFixed= true;
      }
      if (! isFixed) {
	qVarIndex(jj)= i;
	jj++;
      }
    }
    if (! (qVarIndex.size() == gSizeInitial)) {
      cout << "Assembling System not possible! "<<endl;
      cout << "  number of constriants    : "<<gSizeInitial<<endl;
      cout << "  dimension of non fixed q : "<<qVarIndex.size()<<endl<<endl;
    }
    else {
      Vec z(getzSize());
      initz(z);  // x=x0; .. u=u0
      Vec qVar0(qVarIndex.size()); 									  // Anfangswert fuer Newton
      for (int i=0; i<qVar0.size(); i++)  qVar0(i)= q(qVarIndex(i));
      MBSCalculateConstraints_g funcConstraints_g(this,t0);
      cout<<"    starting with NormInf(g)  :"<<nrmInf(funcConstraints_g(qVar0))<<endl;
      MultiDimNewtonMethod solverNewton(&funcConstraints_g);
      solverNewton.setTol(1e-13);
      solverNewton.setMaxIter(100);
      Vec qVarErg = solverNewton.slv(qVar0);
      if (solverNewton.getInfo()== -1) {
	for (int j=0; j<qVar0.size(); j++) {
	  Vec qVar0Tmp =qVar0.copy();
	  if (qVar0Tmp(j)) qVar0Tmp*= 1.1;
	  else qVar0Tmp(j) = 0.1;
	  qVarErg = solverNewton.slv(qVar0Tmp); 
	  if (solverNewton.getInfo()==0) {
	    cout<<"    starting conditions newton solver changed for convergence."<<endl;
	    break;
	  }
	}
      }
      if (solverNewton.getInfo()) cout <<"WARNING: assembling system failed ! ("<<solverNewton.getIter()<<" iterations)"<<endl;
      else  {
	cout << "    finished at NormInf(g)  :"<<nrmInf(funcConstraints_g(qVarErg))<<endl;
	cout << "Assembling System succesfull ("<<solverNewton.getIter()<<" iterations; "<<qVar0.size()<<" parameters)"<<endl<<endl;
        for (int i=0; i<qVarIndex.size(); i++) q(qVarIndex(i)) = qVarErg(i);
        for (vector<Object*>::iterator io = objects.begin(); io != objects.end(); ++io) (*io)->storeq0();
      }
    }
  }

  void MultiBodySystem::assembleSystem_calculateg(const Vec & qVar, Vec &g_constrain, double t0) { 
    g_constrain.resize(qVarIndex.size());
    for (int i=0; i<qVarIndex.size(); i++) q(qVarIndex(i)) = qVar(i);
    updateKinematics(t0);
    updateLinksStage1(t0);
    int jj=0;
    for(vector<Link*>::iterator il = linksInitialActive.begin(); il != linksInitialActive.end(); ++il)  {
      g_constrain(jj,jj+(*il)->getgSize()-1) = (*il)->getg();
      jj += (*il)->getgSize(); 
    }


  }

Vec MultiBodySystem::getAllUnilateralla() {
    int sizeAll =0;
    for(vector<Link*>::iterator il = linkSetValuedUnilateral.begin(); il != linkSetValuedUnilateral.end(); ++il) 
      sizeAll+= (*il)->getlaSize();
    Vec laAll(sizeAll);
    int i = 0;
    int sizelink;
    for(vector<Link*>::iterator il = linkSetValuedUnilateral.begin(); il != linkSetValuedUnilateral.end(); ++il) {
      sizelink = (*il)->getlaSize();
      laAll(i,i+sizelink-1) = (*il)->getla();
      i+= sizelink;
    }
  return laAll;
  }

  void MultiBodySystem::setAllUnilateralla(const Vec laAllUni) {
    int sizeAll = laAllUni.size();
    int i = 0;
    int sizelink;
    for(vector<Link*>::iterator il = linkSetValuedUnilateral.begin(); il != linkSetValuedUnilateral.end(); ++il) {
      sizelink = (*il)->getlaSize();
      if (i+sizelink <= sizeAll)
      (*il)->getla() = laAllUni(i,i+sizelink-1);
      i+= sizelink;
    }
  }

Vec MultiBodySystem::getAllBilateralla() {
    int sizeAll =0;
    for(vector<Link*>::iterator il = linkSetValuedBilateral.begin(); il != linkSetValuedBilateral.end(); ++il) 
      sizeAll+= (*il)->getlaSize();
    Vec laAll(sizeAll);
    int i = 0;
    int sizelink;
    for(vector<Link*>::iterator il = linkSetValuedBilateral.begin(); il != linkSetValuedBilateral.end(); ++il) {
      sizelink = (*il)->getlaSize();
      laAll(i,i+sizelink-1) = (*il)->getla();
      i+= sizelink;
    }
  return laAll;
  }

  void MultiBodySystem::setAllBilateralla(const Vec laAllBi) {
    int sizeAll = laAllBi.size();
    int i = 0;
    int sizelink;
    for(vector<Link*>::iterator il = linkSetValuedBilateral.begin(); il != linkSetValuedBilateral.end(); ++il) {
      sizelink = (*il)->getlaSize();
      if (i+sizelink <= sizeAll)
      (*il)->getla() = laAllBi(i,i+sizelink-1);
      i+= sizelink;
    }
  }


}
