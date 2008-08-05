/* Copyright (C) 2004-2008  Martin Förg

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
 *
 */

#include <config.h>
#include "subsystem.h"
#include "extra_dynamic_interface.h"
#include "link.h"
#include "contour.h"
#include "coordinate_system.h"

namespace MBSim {

  Subsystem::Subsystem(const string &name) : Object(name), gSize(0), laSize(0), rFactorSize(0), svSize(0), svInd(0), nHSLinkSetValuedFixed(0), nHSLinkSingleValuedFixed(0) {
    CoordinateSystem *cosy = new CoordinateSystem("O");
    Object::addCoordinateSystem(cosy);
    cosy->setWrOP(Vec(3));
    cosy->setAWP(SqrMat(3,EYE));
  }

  Subsystem::~Subsystem() {
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      delete *i;
    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      delete *i;
    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i != EDI.end(); ++i)
      delete *i;
    for(vector<DataInterfaceBase*>::iterator i = DIB.begin(); i != DIB.end(); ++i)
      delete *i;
    for(vector<HitSphereLink*>::iterator i =  HSLink.begin(); i != HSLink.end(); ++i)
      delete *i;
  }

  void Subsystem::init() {
    for(unsigned i=0; i<object.size(); i++)
      object[i]->init();

    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) {
      (**i).init();
      if(!(*i)->getHitSphereCheck()) {
	if((*i)->isSetValued()) {
	  nHSLinkSetValuedFixed++;
	  linkSetValued.push_back(*i);
	} else {
	  nHSLinkSingleValuedFixed++;
	  linkSingleValued.push_back(*i);
	}
      }
    }

    Object::init();
  }

  void Subsystem::initz() {
    Object::initz();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->initz();
  }

  void Subsystem::closePlotFiles() {
    Object::closePlotFiles();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->closePlotFiles();
    for(unsigned i=0; i<link.size(); i++)
      link[i]->closePlotFiles();
    for(unsigned i=0; i<EDI.size(); i++)
      EDI[i]->closePlotFiles();
  }

  void Subsystem::initPlotFiles() {
    Object::initPlotFiles();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->initPlotFiles();
    for(unsigned i=0; i<link.size(); i++)
      link[i]->initPlotFiles();
    for(unsigned i=0; i<EDI.size(); i++)
      EDI[i]->initPlotFiles();

    //   // Energieterme
    //   if(plotLevel>=3) {
    //     plotfile <<"# " << plotNr++ << ": T" << endl;
    //     plotfile <<"# " << plotNr++ << ": V" << endl;
    //     plotfile <<"# " << plotNr++ << ": E" << endl;
    //   }   
  }

  void Subsystem::plot(double t, double dt) {
    Object::plot(t,dt);
    for(unsigned i=0; i<object.size(); i++)
      object[i]->plot(t,dt);
    for(unsigned i=0; i<link.size(); i++)
      link[i]->plot(t,dt);
    for(unsigned i=0; i<EDI.size(); i++)
      EDI[i]->plot(t,dt);

    //   if(plotLevel>=3) {
    //     double Ttemp = this->computeKineticEnergy();
    //     double Vtemp = this->computePotentialEnergy();
    //     plotfile<<" "<< Ttemp;
    //     plotfile<<" "<< Vtemp;
    //     plotfile<<" "<< Ttemp + Vtemp;
    //   }

  }

  void Subsystem::plotParameters() {
    Object::plotParameters();
    // all Objects of MultibodySystem
    if(object.size()>0) {
      parafile << "\nObjects:" << endl;
      for(vector<Object*>::iterator i = object.begin();  i != object.end();  ++i)
	parafile << "#  " << (**i).getName() << endl;
    }
    for(unsigned i=0; i<object.size(); i++)
      object[i]->plotParameters();
    for(unsigned i=0; i<link.size(); i++)
      link[i]->plotParameters();
    for(unsigned i=0; i<EDI.size(); i++)
      EDI[i]->plotParameters();
  }

  void Subsystem::addSubsystem(Subsystem *system, const Vec &RrRC, const SqrMat &ARC, const CoordinateSystem* refCoordinateSystem) 
  {
    //object->setFullName(getFullName()+"."+subsystem->getFullName());
    subsystem.push_back(system);
    object.push_back(system);
    //object->setMbs(this);
    system->setParent(this);

    int i = 0;
    if(refCoordinateSystem)
      i = portIndex(refCoordinateSystem);

    system->getCoordinateSystems()[0]->setWrOP(port[i]->getWrOP() + port[i]->getAWP()*RrRC);
    system->getCoordinateSystems()[0]->setAWP(port[i]->getAWP()*ARC);
    for(unsigned int i=1; i<system->getCoordinateSystems().size(); i++) {
      system->getCoordinateSystems()[i]->setWrOP(system->getCoordinateSystems()[0]->getWrOP() + system->getCoordinateSystems()[0]->getAWP()*system->getCoordinateSystems()[i]->getWrOP() );
      system->getCoordinateSystems()[i]->setAWP(system->getCoordinateSystems()[0]->getAWP()*system->getCoordinateSystems()[i]->getAWP());
    }

    for(unsigned int i=0; i<system->getContours().size(); i++) {
      system->getContours()[i]->setWrOP(system->getCoordinateSystems()[0]->getWrOP() + system->getCoordinateSystems()[0]->getAWP()*system->getContours()[i]->getWrOP() );
      system->getContours()[i]->setAWC(system->getCoordinateSystems()[0]->getAWP()*system->getContours()[i]->getAWC());
    }
  }

  void Subsystem::addObject(Object *obj) 
  {
    // ADDOBJECT adds an object
    if(getObject(obj->getFullName(),false)) {
      cout << "Error: The Subsystem " << name << " can only comprise one Object by the name " <<  obj->getFullName() << "!" << endl;
      assert(getObject(obj->getFullName(),false) == NULL); 
    }
    obj->setFullName(getFullName()+"."+obj->getFullName());
    object.push_back(obj);
    //obj->setMbs(this);
    obj->setParent(this);
  }

  void Subsystem::addLink(Link *lnk) {
    if(getLink(lnk->getFullName(),false)) {
      cout << "Error: The Subsystem " << name << " can only comprise one Link by the name " <<  lnk->getFullName() << "!" << endl;
      assert(getLink(lnk->getFullName(),false) == NULL);
    }
    link.push_back(lnk);
    //lnk->setMbs(this);
    lnk->setParent(this);
    lnk->setFullName(getFullName()+"."+lnk->getFullName());

  }

  Object* Subsystem::getObject(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<object.size(); i++) {
      //cout << object[i]->getName() << " " << name << endl;
      if(object[i]->getName() == name)
	return object[i];
    }
    for(i=0; i<object.size(); i++) {
      //cout << object[i]->getFullName() << " " << name << endl;
      if(object[i]->getFullName() == name)
	return object[i];
    }
    if(check){
      if(!(i<object.size())) cout << "Error: The Subsystem " << this->name <<" comprises no object " << name << "!" << endl; 
      assert(i<object.size());
    }
    //    else return NULL;
    return NULL;
  }

  Link* Subsystem::getLink(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<link.size(); i++) {
      //cout << link[i]->getName() << " " << name << endl;
      if(link[i]->getName() == name)
	return link[i];
    }
    for(i=0; i<link.size(); i++) {
      //cout << link[i]->getFullName() << " " << name << endl;
      if(link[i]->getFullName() == name)
	return link[i];
    }
    if(check){
      if(!(i<link.size())) cout << "Error: The Subsystem " << this->name <<" comprises no link " << name << "!" << endl; 
      assert(i<link.size());
    }
    return NULL;
    //    else return NULL;
  }


  void Subsystem::updateKinematics(double t) {
    //for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) (*i)->updateKinematics(t);
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateKinematics(t);
  }

  void Subsystem::updateLinksStage1(double t) {
    if(!HSLink.empty()) {
      linkSingleValued.erase(linkSingleValued.begin()+nHSLinkSingleValuedFixed,linkSingleValued.end());
      linkSetValued.erase(linkSetValued.begin()+nHSLinkSetValuedFixed,linkSetValued.end());
      for(vector<HitSphereLink*>::iterator iHS = HSLink.begin(); iHS != HSLink.end(); ++iHS) (*iHS)->checkActive();
    }

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateLinksStage1(t);
    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i != EDI.end(); ++i) 
      (*i)->updateStage1(t);
    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i) 
      (*i)->updateStage1(t);
    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (*i)->updateStage1(t);
  }

  void Subsystem::updateLinksStage2(double t) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateLinksStage2(t);
    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i != EDI.end(); ++i) 
      (*i)->updateStage2(t);
    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i) 
      (*i)->updateStage2(t);
    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->updateStage2(t);
  }

  void Subsystem::updateStopVector(double t) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateStopVector(t);
    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (*i)->updateStopVector(t); 
  }   

  void Subsystem::updateh(double t) {
    //for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) (*i)->updateh(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateh(t);

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updateh(t);
  }

  void Subsystem::updateT(double t) {
    //for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) (*i)->updateT(t);
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateT(t);
  }

  void Subsystem::updateM(double t) {
    //for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) (*i)->updateM(t);
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updateM(t);
  }

  void Subsystem::updateW(double t) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updateW(t);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).updateW(t);
  }

  void Subsystem::updateb(double t) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) (*i)->updateb(t);
    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).updateb(t);
  }

  void Subsystem::updater(double t) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i)
      (**i).updater(t);
    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).updater(t);
  }

  void Subsystem::updatedx(double t, double dt) {
    // for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) (*i)->updatedx(t,dt);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatedx(t,dt);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatedx(t,dt);

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatedx(t,dt);

  }

  void Subsystem::updatedq(double t, double dt) {
    //for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) (*i)->updatedq(t,dt);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updatedq(t,dt);

  }

  void Subsystem::updateqRef() {

    Object::updateqRef();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateqRef();
  }

  void Subsystem::updateqdRef() {

    Object::updateqdRef();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateqdRef();
  }

  void Subsystem::updateuRef() {

    Object::updateuRef();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateuRef();
  }

  void Subsystem::updateudRef() {

    Object::updateudRef();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateudRef();
  }

  void Subsystem::updatexRef() {

    Object::updatexRef();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatexRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexRef();

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatexRef();

  }

  void Subsystem::updatexdRef() {

    Object::updatexdRef();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatexdRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexdRef();

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatexdRef();

  }

  void Subsystem::updatezRef() {

    Object::updatezRef();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatezRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexRef();

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatexRef();

  }

  void Subsystem::updatezdRef() {

    Object::updatezdRef();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatezdRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexdRef();

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatexdRef();

  }

  void Subsystem::updatehRef() {

    Object::updatehRef();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatehRef();

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updatehRef();

  }

  void Subsystem::updaterRef() {

    Object::updaterRef();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updaterRef();

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updaterRef();

  }

  void Subsystem::updatefRef() {

    Object::updatefRef();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatefRef();
  }

  void Subsystem::updatesvRef() {

    sv >> parent->getsv()(svInd,svInd+svSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatesvRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatesvRef();
  }

  void Subsystem::updatejsvRef() {

    jsv >> parent->getjsv()(svInd,svInd+svSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatejsvRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatejsvRef();
  }

  void Subsystem::updateMRef() {

    Object::updateMRef();

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateMRef();
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateMRef();
  }

  void Subsystem::updateLLMRef() {

    Object::updateLLMRef();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateLLMRef();
  }

  void Subsystem::updateTRef() {

    Object::updateTRef();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateTRef();
  }

  void Subsystem::updategRef() {
    g >> parent->getg()(gInd,gInd+gSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updategRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      if((*i)->isSetValued()) 
	(**i).updategRef();
  }

  void Subsystem::updateWRef() {
    W >> parent->getW()(Index(uInd,uInd+uSize-1),Index(laInd,laInd+laSize-1));

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateWRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updateWRef();
  }

  void Subsystem::updatelaRef() {
    la >> parent->getla()(laInd,laInd+laSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatelaRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatelaRef();
  }

  void Subsystem::updategdRef() {
    gd >> parent->getgd()(laInd,laInd+laSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updategdRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updategdRef();
  }

  void Subsystem::updatebRef() {
    b >> parent->getb()(laInd,laInd+laSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatebRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatebRef();
  }

  void Subsystem::updatesRef() {
    s >> parent->getgd()(laInd,laInd+laSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatesRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatesRef();
  }
  void Subsystem::updateresRef() {
    res >> parent->getgd()(laInd,laInd+laSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateresRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updateresRef();
  }

  void Subsystem::updaterFactorRef() {
    rFactor >> parent->getrFactor()(rFactorInd,rFactorInd+rFactorSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updaterFactorRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updaterFactorRef();
  }

  void Subsystem::updateRef() {
    //cout << parent->getW()<<endl;
    //cout << parent->getla()<<endl;
    //cout << uInd<<endl;
    //cout << uSize<<endl;
    //cout << laInd<<endl;
    //cout << laSize<<endl;
    //cout << W <<endl;
    W.resize() >> parent->getW()(Index(uInd,uInd+uSize-1),Index(laInd,laInd+laSize-1));
    la.resize() >> parent->getla()(laInd,laInd+laSize-1);
    gd.resize() >> parent->getgd()(laInd,laInd+laSize-1);
    b.resize() >> parent->getb()(laInd,laInd+laSize-1);
    s.resize() >> parent->gets()(laInd,laInd+laSize-1);
    res.resize() >> parent->getres()(laInd,laInd+laSize-1);
    rFactor.resize() >> parent->getrFactor()(rFactorInd,rFactorInd+rFactorSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateRef();

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updateRef();
    //updateWRef();
    //updatelaRef();
    //updategdRef();
    //updatebRef();
    //updatesRef();
    //updateresRef();
    //updaterFactorRef();
  }


  void Subsystem::addCoordinateSystem(CoordinateSystem* cosy, const Vec &RrRC, const SqrMat &ARC, const CoordinateSystem* refCoordinateSystem) {

    Object::addCoordinateSystem(cosy);
    int i = 0;
    if(refCoordinateSystem)
      i = portIndex(refCoordinateSystem);

    cosy->setWrOP(port[i]->getWrOP() + port[i]->getAWP()*RrRC);
    cosy->setAWP(port[i]->getAWP()*ARC);
  }

  void Subsystem::addCoordinateSystem(const string &str, const Vec &SrSK, const SqrMat &ASK, const CoordinateSystem* refCoordinateSystem) {
    addCoordinateSystem(new CoordinateSystem(str),SrSK,ASK,refCoordinateSystem);
  }

  void Subsystem::addContour(Contour* contour, const Vec &RrRC, const SqrMat &ARC, const CoordinateSystem* refCoordinateSystem) {

    Object::addContour(contour);
    int i = 0;
    if(refCoordinateSystem)
      i = portIndex(refCoordinateSystem);

    contour->setWrOP(port[i]->getWrOP() + port[i]->getAWP()*RrRC);
    contour->setAWC(port[i]->getAWP()*ARC);
  }

  void Subsystem::calchSize() {

    hSize = uSize;
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->sethSize((*i)->getuSize());
      (*i)->sethInd((*i)->getuInd());
      (*i)->calchSize();
    }
  }

  void Subsystem::calcSize() {

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->calcSize();
      (*i)->setqInd(qSize);
      (*i)->setuInd(uSize);
      (*i)->setxInd(xSize);
      qSize += (*i)->getqSize();
      uSize += (*i)->getuSize();
      xSize += (*i)->getxSize();
    }
  }

  void Subsystem::calclaSize() {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
      (*i)->calclaSize();
      (*i)->setgInd(gSize);
      (*i)->setlaInd(laSize);
      (*i)->setrFactorInd(rFactorSize);
      gSize += (*i)->getgSize();
      laSize += (*i)->getlaSize();
      rFactorSize += (*i)->getrFactorSize();

      (*i)->setxInd(xSize);
      xSize += (*i)->getxSize();

      (*i)->setsvInd(svSize);
      svSize += (*i)->getsvSize();
    }

    for(vector<Link*>::iterator il = link.begin(); il != link.end(); ++il) {
      (*il)->calcSize();
      if((*il)->isSetValued()) {
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

    for( vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) {
      (*i)->setxInd(xSize);
      xSize += (*i)->getxSize();
    }

  }

  void Subsystem::checkActiveConstraints() {

    linkSetValuedActive.clear();
    laSize = 0;
    rFactorSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
      (*i)->checkActiveConstraints();
      (*i)->setlaInd(laSize);
      (*i)->setrFactorInd(rFactorSize);
      laSize += (*i)->getlaSize();
      rFactorSize += (*i)->getrFactorSize();
    }
    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      if((*i)->isActive()) {
	linkSetValuedActive.push_back(*i);
	(*i)->setlaInd(laSize);
	(*i)->setrFactorInd(rFactorSize);
	laSize += (*i)->getlaSize();
	rFactorSize += (*i)->getrFactorSize();
      }
    }

    //updateRef();

    //for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
    //(**i).updateRef();

    //for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
    //(*i)->updateRef();
  }

  HitSphereLink* Subsystem::getHitSphereLink(Object* obj0, Object* obj1) {
    // test for existing HitSphereLinks
    for(vector<HitSphereLink*>::iterator hsl = HSLink.begin();hsl < HSLink.end();hsl++)
      if((*hsl)->getObject(0) == obj0 && (*hsl)->getObject(1) == obj1 || (*hsl)->getObject(0) == obj1 && (*hsl)->getObject(1) == obj0)
	return  (*hsl);

    //     cout << "Creating new HitSphereLink for " << obj0->getName() << "<->" << obj1->getName() << endl;

    // create new if none is found
    HitSphereLink *HSLnk = new HitSphereLink();
    HSLink.push_back(HSLnk);
    return HSLnk;
  }

  ExtraDynamicInterface* Subsystem::getEDI(const string &name,bool check) {
    // GETEDI returns an extra dynamic interface
    unsigned int i;
    for(i=0; i<EDI.size(); i++) {
      if(EDI[i]->getName() == name) return EDI[i];
    }
    for(i=0; i<EDI.size(); i++) {
      if(EDI[i]->getFullName() == name) return EDI[i];
    }
    if(check) {
      if(!(i<EDI.size())) cout << "Error: The Subsystem " << this->name <<" comprises no EDI " << name << "!" << endl; 
      assert(i<EDI.size());
    }
    return NULL; 
  }    

  void Subsystem::addEDI(ExtraDynamicInterface *edi_) {
    if(getEDI(edi_->getFullName(),false)) {
      cout << "Error: The Subsystem " << name << " can only comprise one ExtraDynamicInterface by the name " <<  edi_->getFullName() << "!" << endl;
      assert(getEDI(edi_->getFullName(),false) == NULL);
    }
    EDI.push_back(edi_);
    //edi_->setMbs(this);
    edi_->setParent(this);
    edi_->setFullName(getFullName()+"."+edi_->getFullName());
  }


  DataInterfaceBase* Subsystem::getDataInterfaceBase(const string &name_, bool check) {
    unsigned int i;
    for(i=0; i<DIB.size(); i++) {
      if(DIB[i]->getName() == name_ || DIB[i]->getName()== fullName+"."+name_ || DIB[i]->getName() == name_+".SigOut" || DIB[i]->getName()== fullName+"."+name_+".SigOut")
	return DIB[i];
    }
    if(check){
      if(!(i<DIB.size())) cout << "Error: The Subsystem " << name <<" comprises no DIB " << name_ << "!" << endl; 
      assert(i<DIB.size());
    } 
    return NULL;
  }    

  void Subsystem::addDataInterfaceBase(DataInterfaceBase* dib_) {
    if(getDataInterfaceBase(dib_->getName(),false)) {
      cout << "Error: The Subsystem " << name << " can only comprise one DataInterfaceBase by the name " <<  dib_->getName() << "!" << endl;
      assert(getDataInterfaceBase(dib_->getName(),false) == NULL);
    }
    DIB.push_back(dib_);
    dib_->setName(getFullName()+"."+dib_->getName());
  }

  int Subsystem::solveFixpointSingle(double dt) {
    // SOLVEFIXEDPOINTSINGLE solves constraint equations with single step iteration
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) (*i)->solveFixpointSingle(dt); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) (*i)->projectGS(dt);

    return 0;
  }

  void Subsystem::updaterFactors() {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updaterFactors(); 
    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updaterFactors();
  }

  void Subsystem::checkForTermination(double dt) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->checkForTermination(dt); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (**i).checkForTermination(dt);
      //  if(term == false) return;
    }
  }

  // CoordinateSystem* Subsystem::getCoordinateSystem(const string &name,bool check) {
  // unsigned int i;
  // for(i=0; i<port.size(); i++) {
  //   if(port[i]->getName() == name || port[i]->getFullName()== name) return port[i];
  // }
  // if(check){
  //   if(!(i<port.size())) cout << "Error: The Subsystem " << this->name <<" comprises no port " << name << "!" << endl; 
  //   assert(i<port.size());
  // }
  //   else return NULL;
  // return NULL;
  // }
  //
  //
  //Contour* Subsystem::getContour(const string &name,bool check) {
  //  unsigned int i;
  //  for(i=0; i<contour.size(); i++) {
  //    if(contour[i]->getName() == name || contour[i]->getFullName()== name) return contour[i];
  //  }
  //  if(check){
  //    if(!(i<contour.size())) cout << "Error: The Subsystem " << this->name <<" comprises no contour " << name << "!" << endl; 
  //    assert(i<contour.size());
  //  }
  //    else return NULL;
  //  return NULL;
  //}

}
