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
#include <mbsim/subsystem.h>
#include <mbsim/extra_dynamic_interface.h>
#include <mbsim/link.h>
#include <mbsim/contour.h>
#include <mbsim/frame.h>
#include <mbsim/class_factory.h>
#include <mbsim/multi_body_system.h>
#include <hdf5serie/fileserie.h>

#include "compatibility_classes/tree_rigid.h"
#include "compatibility_classes/body_rigid.h"

namespace MBSim {

  Subsystem::Subsystem(const string &name) : Element(name), parent(0), qSize(0), qInd(0), xSize(0), xInd(0), gSize(0), gInd(0), gdSize(0), gdInd(0), laSize(0), laInd(0), rFactorSize(0), rFactorInd(0), svSize(0), svInd(0), q0(qSize), u0(0), x0(xSize) {

    uSize[0] = 0;
    uSize[1] = 0;
    uInd[0] = 0;
    uInd[1] = 0;
    hSize[0] = 0;
    hSize[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;

    addFrame(new Frame("I"));

    IrOK.push_back(Vec(3));
    AIK.push_back(SqrMat(3,EYE));

    port[0]->setPosition(Vec(3));
    port[0]->setOrientation(SqrMat(3,EYE));
#ifdef HAVE_AMVISCPPINTERFACE
    if(amvisGrp) amvisGrp=0;
#endif

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
#ifdef HAVE_AMVISCPPINTERFACE
    delete amvisGrp;
#endif
  }

  int Subsystem::gethInd(Subsystem* sys, int i) {
    return (this == sys) ? 0 : ((parent == this) ? hInd[i] : hInd[i] + parent->gethInd(sys,i));
  }

  void Subsystem::setMultiBodySystem(MultiBodySystem* sys) {
    Element::setMultiBodySystem(sys);
    for(unsigned i=0; i<subsystem.size(); i++)
      subsystem[i]->setMultiBodySystem(sys);

    for(unsigned i=0; i<object.size(); i++)
      object[i]->setMultiBodySystem(sys);

    for(unsigned i=0; i<link.size(); i++)
      link[i]->setMultiBodySystem(sys);

    for (unsigned i=0; i<EDI.size(); i++)
      EDI[i]->setMultiBodySystem(sys);
  }

  void Subsystem::setlaIndMBS(int laIndParent) {
    int newlaInd = laInd + laIndParent;
    for(unsigned i=0; i<subsystem.size(); i++)
      subsystem[i]->setlaIndMBS(newlaInd);

    for(unsigned i=0; i<link.size(); i++)
      link[i]->setlaIndMBS(newlaInd);
  }

  void Subsystem::setlaTol(double tol) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i!= subsystem.end(); ++i)
      (**i).setlaTol(tol);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setlaTol(tol);
  }

  void Subsystem::setLaTol(double tol) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i!= subsystem.end(); ++i)
      (**i).setLaTol(tol);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setLaTol(tol);
  }

  void Subsystem::setgdTol(double tol) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i!= subsystem.end(); ++i)
      (**i).setgdTol(tol);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setgdTol(tol);
  }

  void Subsystem::setgddTol(double tol) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i!= subsystem.end(); ++i)
      (**i).setgddTol(tol);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setgddTol(tol);
  }

  void Subsystem::setrMax(double rMax) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i!= subsystem.end(); ++i)
      (**i).setrMax(rMax);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setrMax(rMax);
  }

  void Subsystem::setScaleTolQ(double scaleTolQ) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i!= subsystem.end(); ++i)
      (**i).setScaleTolQ(scaleTolQ);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setScaleTolQ(scaleTolQ);
  }

  void Subsystem::setScaleTolp(double scaleTolp) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i!= subsystem.end(); ++i)
      (**i).setScaleTolp(scaleTolp);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setScaleTolp(scaleTolp);
  }

  void Subsystem::setFullName(const string &str) {
    Element::setFullName(str);
    for(unsigned i=0; i<subsystem.size(); i++)
      subsystem[i]->setFullName(getFullName() + "." + subsystem[i]->getName());

    for(unsigned i=0; i<object.size(); i++)
      object[i]->setFullName(getFullName() + "." + object[i]->getName());

    for(unsigned i=0; i<link.size(); i++)
      link[i]->setFullName(getFullName() + "." + link[i]->getName());

    for (unsigned i=0; i<EDI.size(); i++)
      EDI[i]->setFullName(getFullName() + "." + EDI[i]->getName());

    for(unsigned i=0; i<port.size(); i++)
      port[i]->setFullName(getFullName() + "." + port[i]->getName());
    for(unsigned i=0; i<contour.size(); i++)
      contour[i]->setFullName(getFullName() + "." + contour[i]->getName());
  }

  void Subsystem::preinit() {

    for(unsigned int i=0; i<subsystem.size(); i++) 
      subsystem[i]->preinit();

    for(unsigned i=0; i<object.size(); i++)
      object[i]->preinit();

    for(unsigned i=0; i<link.size(); i++)
      link[i]->preinit();

    for (unsigned i=0; i<EDI.size(); i++)
      EDI[i]->preinit();
  }

  void Subsystem::init() {

    // if(mbs != this) {
    //   port[0]->setPosition(portParent->getPosition() +  portParent->getOrientation()*PrPK);
    //   port[0]->setOrientation(portParent->getOrientation()*APK);
    // }

    // Kinematik der anderen KOSY (außer Ursprung- und Referenz-) updaten, ausgehend vom Ursprung-KOSY
    for(unsigned int i=1; i<port.size(); i++) {
      port[i]->setPosition(port[0]->getPosition() + port[0]->getOrientation()*IrOK[i]);
      port[i]->setOrientation(port[0]->getOrientation()*AIK[i]);
    }
    // Kinematik der Konturen updaten, ausgehend vom Ursprung-KOSY
    for(unsigned int i=0; i<contour.size(); i++) {
      contour[i]->setWrOP(port[0]->getPosition() + port[0]->getOrientation()*IrOC[i]);
      contour[i]->setAWC(port[0]->getOrientation()*AIC[i]);
      contour[i]->init();
    }
    // Kinematik der Konturen updaten, ausgehend vom Ursprung-KOSY
    for(unsigned int i=0; i<subsystem.size(); i++) {
      subsystem[i]->getFrame("I")->setPosition(port[0]->getPosition() + port[0]->getOrientation()*IrOS[i]);
      subsystem[i]->getFrame("I")->setOrientation(port[0]->getOrientation()*AIS[i]);
      subsystem[i]->init();
    }

    for(unsigned i=0; i<object.size(); i++)
      object[i]->init();

    for(unsigned i=0; i<link.size(); i++)
      link[i]->init();

    for (unsigned i=0; i<EDI.size(); i++)
      EDI[i]->init();
  }

  void Subsystem::initz() {
    q = q0;
    u = u0;
    x = x0;
    for(unsigned i=0; i<subsystem.size(); i++)
      subsystem[i]->initz();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->initz();
    for(unsigned i=0; i<link.size(); i++)
      link[i]->initz();
    for(unsigned i=0; i<EDI.size(); i++)
      EDI[i]->initz();
  }


  
  void Subsystem::setUpLinks() {
    for(unsigned int i=0; i<subsystem.size(); i++) 
      subsystem[i]->setUpLinks();

    for(unsigned int i=0; i<link.size(); i++) {
      if(link[i]->isSetValued()) {
	linkSetValued.push_back(link[i]);
	linkSetValuedActive.push_back(link[i]);
      }
    else 
      linkSingleValued.push_back(link[i]);
    }
  }

  void Subsystem::addSubsystem(Subsystem *sys) {
    // ADDOBJECT adds an subsystem
    if(getSubsystem(sys->getName(),false)) {
      cout << "Error: The Subsystem " << name << " can only comprise one Object by the name " <<  sys->getName() << "!" << endl;
      assert(getSubsystem(sys->getName(),false) == NULL); 
    }
    subsystem.push_back(sys);
    sys->setParent(this);
  }

  void Subsystem::addObject(Object *obj) {
    // ADDOBJECT adds an object
    if(getObject(obj->getName(),false)) {
      cout << "Error: The Subsystem " << name << " can only comprise one Object by the name " <<  obj->getName() << "!" << endl;
      assert(getObject(obj->getName(),false) == NULL); 
    }
    object.push_back(obj);
    obj->setParent(this);
  }

  void Subsystem::addLink(Link *lnk) {
    if(getLink(lnk->getName(),false)) {
      cout << "Error: The Subsystem " << name << " can only comprise one Link by the name " <<  lnk->getName() << "!" << endl;
      assert(getLink(lnk->getName(),false) == NULL);
    }

    link.push_back(lnk);
    lnk->setParent(this);
  }

  Subsystem* Subsystem::getSubsystem(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<subsystem.size(); i++) {
      if(subsystem[i]->getName() == name)
	return subsystem[i];
    }
    if(check){
      if(!(i<subsystem.size())) cout << "Error: The Subsystem " << this->name <<" comprises no subsystem " << name << "!" << endl; 
      assert(i<subsystem.size());
    }
    return NULL;
  }

  Object* Subsystem::getObject(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<object.size(); i++) {
      if(object[i]->getName() == name)
	return object[i];
    }
    if(check){
      if(!(i<object.size())) cout << "Error: The Subsystem " << this->name <<" comprises no object " << name << "!" << endl; 
      assert(i<object.size());
    }
    return NULL;
  }

  Link* Subsystem::getLink(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<link.size(); i++) {
      if(link[i]->getName() == name)
	return link[i];
    }
    if(check){
      if(!(i<link.size())) cout << "Error: The Subsystem " << this->name <<" comprises no link " << name << "!" << endl; 
      assert(i<link.size());
    }
    return NULL;
  }



  void Subsystem::updateg(double t) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateg(t);
    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i != EDI.end(); ++i) 
      (*i)->updateg(t);
    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i) 
      (*i)->updateg(t);
    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (*i)->updateg(t);
  }

  void Subsystem::updategd(double t) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updategd(t);
    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i != EDI.end(); ++i) 
      (*i)->updategd(t);
    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i) 
      (*i)->updategd(t);
    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->updategd(t);
  }

  void Subsystem::updateStopVector(double t) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateStopVector(t);
    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (*i)->updateStopVector(t); 
  }   

  void Subsystem::updateh(double t) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updateh(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateh(t);

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updateh(t);
  }

  void Subsystem::updateT(double t) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updateT(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateT(t);
  }

  void Subsystem::updateM(double t) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updateM(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updateM(t);
  }

  void Subsystem::updateW(double t) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updateW(t);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).updateW(t);
  }

  void Subsystem::updateV(double t) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updateV(t);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).updateV(t);
  }

  void Subsystem::updatewb(double t) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatewb(t);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).updatewb(t);
  }

  void Subsystem::updater(double t) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i)
      (**i).updater(t);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).updater(t);
  }

  void Subsystem::updatexd(double t) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updatexd(t);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexd(t);

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatexd(t);
  }

  void Subsystem::updatedx(double t, double dt) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updatedx(t,dt);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatedx(t,dt);

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatedx(t,dt);

  }

  void Subsystem::updatedq(double t, double dt) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i)
      (**i).updatedq(t,dt);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updatedq(t,dt);
  }

  void Subsystem::updateqRef(const Vec &qParent) {

    q >> qParent(qInd,qInd+qSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updateqRef(q);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateqRef(q);
  }

  void Subsystem::updateqdRef(const Vec &qdParent) {

    qd >> qdParent(qInd,qInd+qSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updateqdRef(qd);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateqdRef(qd);
  }

  void Subsystem::updateuRef(const Vec &uParent) {

    u >> uParent(uInd[0],uInd[0]+uSize[0]-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updateuRef(u);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateuRef(u);
  }

  void Subsystem::updateudRef(const Vec &udParent) {

    ud >> udParent(uInd[0],uInd[0]+uSize[0]-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updateudRef(ud);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateudRef(ud);
  }

  void Subsystem::updatexRef(const Vec &xParent) {

    x >> xParent(xInd,xInd+xSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updatexRef(x);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexRef(x);

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatexRef(x);
  }

  void Subsystem::updatexdRef(const Vec &xdParent) {

    xd >> xdParent(xInd,xInd+xSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updatexdRef(xd);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexdRef(xd);

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatexdRef(xd);
  }

  void Subsystem::updatehRef(const Vec &hParent, int j) {

    h.resize() >> hParent(hInd[j],hInd[j]+hSize[j]-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updatehRef(h,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatehRef(h,j);

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updatehRef(h);
  }

  void Subsystem::updaterRef(const Vec &rParent) {

    r >> rParent(hInd[0],hInd[0]+hSize[0]-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updaterRef(r);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updaterRef(r);

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updaterRef(r);
  }

  void Subsystem::updatefRef(const Vec &fParent) {
    f >> fParent(xInd,xInd+xSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updatefRef(f);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatefRef(f);
  }

  void Subsystem::updatesvRef(const Vec &svParent) {

    sv >> svParent(svInd,svInd+svSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatesvRef(sv);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatesvRef(sv);
  }

  void Subsystem::updatejsvRef(const Vector<int> &jsvParent) {

    jsv >> jsvParent(svInd,svInd+svSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatejsvRef(jsv);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatejsvRef(jsv);
  }

  void Subsystem::updateMRef(const SymMat& MParent, int j) {

    M.resize() >> MParent(Index(hInd[j],hInd[j]+hSize[j]-1));

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateMRef(M,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateMRef(M,j);
  }

  void Subsystem::updateLLMRef(const SymMat& LLMParent, int j) {

    LLM.resize() >> LLMParent(Index(hInd[j],hInd[j]+hSize[j]-1));

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateLLMRef(LLM,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateLLMRef(LLM,j);
  }

  void Subsystem::updateTRef(const Mat& TParent) {

    T >> TParent(Index(qInd,qInd+qSize-1),Index(uInd[0],uInd[0]+uSize[0]-1));

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateTRef(T);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateTRef(T);
  }

  void Subsystem::updategRef(const Vec& gParent) {
    g.resize() >> gParent(gInd,gInd+gSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updategRef(g);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updategRef(g);
  }

  void Subsystem::updategdRef(const Vec& gdParent) {

    gd.resize() >> gdParent(gdInd,gdInd+gdSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updategdRef(gd);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updategdRef(gd);
  }

  void Subsystem::updateVRef(const Mat &VParent, int j) {

    V.resize() >> VParent(Index(hInd[j],hInd[j]+hSize[j]-1),Index(laInd,laInd+laSize-1));

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateVRef(V,j);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updateVRef(V,j);
  }

  void Subsystem::updateWRef(const Mat &WParent, int j) {

    W.resize() >> WParent(Index(hInd[j],hInd[j]+hSize[j]-1),Index(laInd,laInd+laSize-1));

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateWRef(W,j);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updateWRef(W,j);
  }

  void Subsystem::updatelaRef(const Vec &laParent) {
    la.resize() >> laParent(laInd,laInd+laSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatelaRef(la);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updatelaRef(la);
  }

  void Subsystem::updatewbRef(const Vec &wbParent) {
    wb.resize() >> wbParent(laInd,laInd+laSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatewbRef(wb);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updatewbRef(wb);
  }

  void Subsystem::updateresRef(const Vec &resParent) {
    res.resize() >> resParent(laInd,laInd+laSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateresRef(res);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updateresRef(res);
  }

  void Subsystem::updaterFactorRef(const Vec &rFactorParent) {

    rFactor.resize() >> rFactorParent(rFactorInd,rFactorInd+rFactorSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updaterFactorRef(rFactor);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updaterFactorRef(rFactor);
  }

  int Subsystem::portIndex(const Frame *port_) const {
    for(unsigned int i=0; i<port.size(); i++) {
      if(port_==port[i])
	return i;
    }
    return -1;
  }

  Frame* Subsystem::getFrame(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<port.size(); i++) {
      if(port[i]->getName() == name)
	return port[i];
    }             
    if(check) {
      if(!(i<port.size())) cout << "Error: The object " << this->name <<" comprises no port " << name << "!" << endl; 
      assert(i<port.size());
    }
    return NULL;
  }

  Contour* Subsystem::getContour(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<contour.size(); i++) {
      if(contour[i]->getName() == name)
	return contour[i];
    }
    if(check) {
      if(!(i<contour.size())) cout << "Error: The object " << this->name <<" comprises no contour " << name << "!" << endl; 
      assert(i<contour.size());
    }
    return NULL;
  }

  void Subsystem::addFrame(Frame* cosy) {

    if(getFrame(cosy->getName(),false)) { //Contourname exists already
      cout << "Error: The Subsystem " << name << " can only comprise one Frame by the name " <<  cosy->getName() << "!" << endl;
      assert(getFrame(cosy->getName(),false)==NULL);
    }
    port.push_back(cosy);
    cosy->setParent(this);
  }

  void Subsystem::addFrame(Frame* cosy, const Vec &RrRK, const SqrMat &ARK, const Frame* refFrame) {

    addFrame(cosy);

    int i = 0;
    if(refFrame)
      i = portIndex(refFrame);

    IrOK.push_back(IrOK[i] + AIK[i]*RrRK);
    AIK.push_back(AIK[i]*ARK);
  }

  void Subsystem::addFrame(const string &str, const Vec &SrSK, const SqrMat &ASK, const Frame* refFrame) {
    addFrame(new Frame(str),SrSK,ASK,refFrame);
  }

  void Subsystem::addContour(Contour* contour_) {

    if(getContour(contour_->getName(),false)) { //Contourname exists already
      cout << "Error: The Subsystem " << name << " can only comprise one Contour by the name " <<  contour_->getName() << "!" << endl;
      assert(getContour(contour_->getName(),false)==NULL);
    }
    contour.push_back(contour_);
    contour_->setParent(this);
  }

  void Subsystem::addContour(Contour* contour, const Vec &RrRC, const SqrMat &ARC, const Frame* refFrame) {

    addContour(contour);

    int i = 0;
    if(refFrame)
      i = portIndex(refFrame);

    IrOC.push_back(IrOK[i] + AIK[i]*RrRC);
    AIC.push_back(AIK[i]*ARC);
  }

  void Subsystem::sethSize(int hSize_, int j) {

    hSize[j] = hSize_;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
      (*i)->sethSize((*i)->getuSize(j),j);
      (*i)->sethInd((*i)->getuInd(j),j);
    }

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->sethSize((*i)->getuSize(j),j);
      (*i)->sethInd((*i)->getuInd(j),j);
    }

   // for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) {
   //   (*i)->sethSize((*i)->getuSize());
   //   (*i)->sethInd((*i)->getuInd());
   // }
  }

  void Subsystem::resizeJacobians(int j) {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->resizeJacobians(j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->resizeJacobians(j);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (*i)->resizeJacobians(j);
  }

  void Subsystem::checkForConstraints() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->checkForConstraints();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->checkForConstraints();
  }

  //void Subsystem::calchSize() {

  //// hSize = 0;

  ////  for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
  ////    (*i)->calchSize();
  ////    (*i)->sethInd(hSize);
  ////    hSize += (*i)->gethSize();
  ////  }
  ////  for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
  ////    (*i)->sethSize((*i)->getuSize());
  ////    (*i)->sethInd(hSize);
  ////    hSize += (*i)->gethSize();
  ////  }
  //  hSize = uSize;
  //  for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
  //    (*i)->sethSize((*i)->getuSize());
  //    (*i)->sethInd((*i)->getuInd());
  //    (*i)->calchSize();
  //  }
  //  for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
  //    (*i)->sethSize((*i)->getuSize());
  //    (*i)->sethInd((*i)->getuInd());
  //    (*i)->calchSize();
  //  }
  //}

  void Subsystem::calcsvSize() {
    svSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
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

  void Subsystem::calcqSize() {
    qSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
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

  void Subsystem::calcuSize(int j) {
    uSize[j] = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
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

  void Subsystem::calcxSize() {
    xSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
      (*i)->calcxSize();
      (*i)->setxInd(xSize);
      xSize += (*i)->getxSize();
    }

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) {
      (*i)->calcxSize();
      (*i)->setxInd(xSize);
      xSize += (*i)->getxSize();
    }

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i != EDI.end(); ++i) {
      (*i)->calcxSize();
      (*i)->setxInd(xSize);
      xSize += (*i)->getxSize();
    }
  }

  void Subsystem::calclaSizeForActiveg() {
    laSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
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

  void Subsystem::calclaSize() {
    laSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
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

  void Subsystem::calcgSizeActive() {
    gSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
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

  void Subsystem::calcgSize() {
    gSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
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

  void Subsystem::calcgdSizeActive() {
    gdSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
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

  void Subsystem::calcgdSize() {
    gdSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
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

  void Subsystem::calcrFactorSize() {
    rFactorSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
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

  bool Subsystem::gActiveChanged() {
    bool changed = false;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      if ((*i)->gActiveChanged())
	changed = true;

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      if ((*i)->gActiveChanged())
	changed = true;

    return changed;
  }

  void Subsystem::checkActiveLinks() {

    linkSetValuedActive.clear();

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->checkActiveLinks();

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      if((*i)->isActive()) {
	linkSetValuedActive.push_back(*i);
      }
    }
  }

  void Subsystem::checkActiveg() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->checkActiveg();

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (*i)->checkActiveg();
  }

  void Subsystem::checkActivegd() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->checkActivegd();

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->checkActivegd();
  }

  void Subsystem::checkActivegdn() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->checkActivegdn();

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->checkActivegdn();
  }

  void Subsystem::checkActivegdd() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->checkActivegdd();

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->checkActivegdd();
  }

  void Subsystem::checkAllgd() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->checkAllgd();

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->checkAllgd();
  }

  void Subsystem::updateCondition() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateCondition();

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (*i)->updateCondition();
  }

  ExtraDynamicInterface* Subsystem::getEDI(const string &name,bool check) {
    // GETEDI returns an extra dynamic interface
    unsigned int i;
    for(i=0; i<EDI.size(); i++) {
      if(EDI[i]->getName() == name) return EDI[i];
    }
    if(check) {
      if(!(i<EDI.size())) cout << "Error: The Subsystem " << this->name <<" comprises no EDI " << name << "!" << endl; 
      assert(i<EDI.size());
    }
    return NULL; 
  }    

  void Subsystem::addEDI(ExtraDynamicInterface *edi_) {
    if(getEDI(edi_->getName(),false)) {
      cout << "Error: The Subsystem " << name << " can only comprise one ExtraDynamicInterface by the name " <<  edi_->getName() << "!" << endl;
      assert(getEDI(edi_->getName(),false) == NULL);
    }
    EDI.push_back(edi_);
    //edi_->setMbs(this);
    edi_->setParent(this);
    //edi_->setFullName(getFullName()+"."+edi_->getFullName());
  }


  DataInterfaceBase* Subsystem::getDataInterfaceBase(const string &name_, bool check) {
    unsigned int i;
    for(i=0; i<DIB.size(); i++) {
      if(DIB[i]->getName() == name_ || DIB[i]->getName() == name_+".SigOut")
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
    // dib_->setName(getFullName()+"."+dib_->getName());
  }

  int Subsystem::solveConstraintsFixpointSingle() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->solveConstraintsFixpointSingle(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveConstraintsFixpointSingle();

    return 0;
  }

  int Subsystem::solveImpactsFixpointSingle() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->solveImpactsFixpointSingle(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveImpactsFixpointSingle();

    return 0;
  }

  int Subsystem::solveConstraintsGaussSeidel() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->solveConstraintsGaussSeidel(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveConstraintsGaussSeidel();

    return 0;
  }

  int Subsystem::solveImpactsGaussSeidel() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->solveImpactsGaussSeidel(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveImpactsGaussSeidel();

    return 0;
  }

  int Subsystem::solveConstraintsRootFinding() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->solveConstraintsRootFinding(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveConstraintsRootFinding();

    return 0;
  }

  int Subsystem::solveImpactsRootFinding() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->solveImpactsRootFinding(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveImpactsRootFinding();

    return 0;
  }

  int Subsystem::jacobianConstraints() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->jacobianConstraints(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->jacobianConstraints();

    return 0;
  }

  int Subsystem::jacobianImpacts() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->jacobianImpacts(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->jacobianImpacts();

    return 0;
  }

  void Subsystem::updaterFactors() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updaterFactors(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updaterFactors();
  }

  void Subsystem::checkConstraintsForTermination() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->checkConstraintsForTermination(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).checkConstraintsForTermination();
  }

  void Subsystem::checkImpactsForTermination() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->checkImpactsForTermination(); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).checkImpactsForTermination();
  }

  void Subsystem::initPlot() {
    if(parent)
      updatePlotFeatures(parent);

    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(separateFilePerSubsystem)==enabled) {
        // create symbolic link in parent plot file if exist
        if(parent) H5Lcreate_external((getFullName()+".mbsim.h5").c_str(), "/",
                                      parent->getPlotGroup()->getId(), name.c_str(),
                                      H5P_DEFAULT, H5P_DEFAULT);
        // create new plot file (cast needed because of the inadequacy of the HDF5 C++ inteface?)
        plotGroup=(H5::Group*)new H5::FileSerie(getFullName()+".mbsim.h5", H5F_ACC_TRUNC);
      }
      else
        plotGroup=new H5::Group(parent->getPlotGroup()->createGroup(name));
      
      H5::SimpleAttribute<string>::setData(*plotGroup, "Description", "Object of class: "+getType());
      plotVectorSerie=NULL;

#ifdef HAVE_AMVISCPPINTERFACE
      amvisGrp=new AMVis::Group();
      amvisGrp->setName(name);
      if(parent) parent->amvisGrp->addObject(amvisGrp);
      if(getPlotFeature(separateFilePerSubsystem)==enabled)
        amvisGrp->setSeparateFile(true);
#endif

      for(unsigned i=0; i<subsystem.size(); i++)
        subsystem[i]->initPlot();
      for(unsigned i=0; i<object.size(); i++)
        object[i]->initPlot();
      for(unsigned i=0; i<link.size(); i++)
        link[i]->initPlot();
      for(unsigned i=0; i<EDI.size(); i++)
        EDI[i]->initPlot();
    }
    plotGroup->flush(H5F_SCOPE_GLOBAL);
  }

  void Subsystem::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      for(unsigned i=0; i<subsystem.size(); i++)
        subsystem[i]->closePlot();
      for(unsigned i=0; i<object.size(); i++)
        object[i]->closePlot();
      for(unsigned i=0; i<link.size(); i++)
        link[i]->closePlot();
      for(unsigned i=0; i<EDI.size(); i++)
        EDI[i]->closePlot();

      Element::closePlot();
    }
  }

  void Subsystem::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      Element::plot(t,dt);

      for(unsigned i=0; i<subsystem.size(); i++)
        subsystem[i]->plot(t,dt);
      for(unsigned i=0; i<object.size(); i++)
        object[i]->plot(t,dt);
      for(unsigned i=0; i<link.size(); i++)
        link[i]->plot(t,dt);
      for(unsigned i=0; i<EDI.size(); i++)
        EDI[i]->plot(t,dt);
    }
  }

}
