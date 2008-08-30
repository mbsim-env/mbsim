/* Copyright (C) 2004-2006  Martin Förg
 
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
#include<config.h>
#include<stdexcept>
#include "object.h"
#include "coordinate_system.h"
#include "contour.h"
#include "link.h"
#include "multi_body_system.h"
#include "eps.h"
#include "subsystem.h"

namespace MBSim {

  Object::Object(const string &name_) : Element(name_), parent(0), qSize(0), uSize(0), xSize(0), hSize(0), qInd(0), uInd(0), xInd(0), hInd(0), q0(qSize), u0(uSize), x0(xSize), WrOHitSphere(3), RHitSphere(0) {}

  void Object::writeq()
  {
//    string fname="PREINTEG/"+fullName+".q0.asc";  
//    ofstream osq(fname.c_str(), ios::out);
//    osq << q;
//    osq.close();
  }
  void Object::readq0()
  {
//    string fname="PREINTEG/"+fullName+".q0.asc";  
//    ifstream isq(fname.c_str());
//    if(isq) isq >> q0;
//    else {cout << "Object " << name << ": No Preintegration Data q0 available. Run Preintegration first." << endl; throw 50;}
//    isq.close();
  }
  void Object::writeu()
  {
 //   string fname="PREINTEG/"+fullName+".u0.asc";  
 //   ofstream osu(fname.c_str(), ios::out);
 //   osu << u;
 //   osu.close();
  }

  void Object::readu0()
  {
 //   string fname="PREINTEG/"+fullName+".u0.asc";  
 //   ifstream isu(fname.c_str());
 //   if(isu) isu >> u0;
 //   else {cout << "Object " << name << ": No Preintegration Data u0 available. Run Preintegration first." << endl; throw 50;}
 //   isu.close();
  }

  void Object::writex()
  {
 //   string fname="PREINTEG/"+fullName+".x0.asc";  
 //   ofstream osx(fname.c_str(), ios::out);
 //   osx << x;
 //   osx.close();
  }

  void Object::readx0()
  {
 //   string fname="PREINTEG/"+fullName+".x0.asc";  
 //   ifstream isx(fname.c_str());
 //   if(isx) isx >> x0;
 //   else {cout << "Object " << name << ": No Preintegration Data x0 available. Run Preintegration first." << endl; throw 50;}
 //   isx.close();
  }

  Object::~Object() {
    // Destructs port and contour pointers
    for(vector<CoordinateSystem*>::iterator i = port.begin(); i != port.end(); ++i) 
      delete *i;
    for(vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i) 
      delete *i;
  }

  void Object::updatezRef() {
    // UPDATEZREF references to positions, velocities and TODO DOC of multibody system parent
    updateqRef();
    updateuRef();
    updatexRef();
  }

  void Object::updatezdRef() 
  {
    // UPDATEZDREF references to differentiated positions, velocities and TODO DOC of multibody system parent
    updateqdRef();
    updateudRef();
    updatexdRef();
  }

  void Object::updateqRef() 
  {
    // UPDATEQREF references to positions of multibody system parent
    q>>(parent->getq()(qInd,qInd+qSize-1));
  }

  void Object::updateqdRef()
  {
    // UPDATEQDREF references to differentiated positions of multibody system parent
    qd>>(parent->getqd()(qInd,qInd+qSize-1));
  }

  void Object::updateuRef()
  {
    // UPDATEUREF references to velocities of multibody system parent
    u>>(parent->getu()(uInd,uInd+uSize-1));
  }

  void Object::updateudRef()
  {
    // UPDATEUDREF references to differentiated velocities of multibody system parent
    ud>>(parent->getud()(uInd,uInd+uSize-1));
  }

  void Object::updatexRef()
  {
    x>>(parent->getx()(xInd,xInd+xSize-1));
  }

  void Object::updatexdRef()
  {
    xd>>(parent->getxd()(xInd,xInd+xSize-1));
  }

  void Object::updatehRef()
  {
    // UPDATEHREF references to smooth force vector of multibody system
    // parent
    //h>>(parent->geth()(uInd,uInd+uSize-1));
    h>>(parent->geth()(hInd,hInd+hSize-1));
  }

  void Object::updaterRef()
  {
    // UPDATERREF references to smooth force vector of multibody system
    // parent
    r>>(parent->getr()(uInd,uInd+uSize-1));
  }

  void Object::updatefRef()
  {
    f>>(parent->getf()(xInd,xInd+xSize-1));
  }

  void Object::updateMRef()
  {
    // UPDATEMREF references to mass matrix of multibody system parent
    Index Iu = Index(hInd,hInd+hSize-1);
    M>>parent->getM()(Iu);
  }

  void Object::updateTRef()
  {
    // UPDATETREF references to T-matrix of multibody system parent
    Index Iu = Index(uInd,uInd+uSize-1);
    Index Iq = Index(qInd,qInd+qSize-1);
    T>>parent->getT()(Iq,Iu);
  }

  void Object::updateLLMRef()
  {
    // UPDATELLMREF references to cholesky decomposition of mass matrix of multibody system parent
    Index Iu = Index(hInd,hInd+hSize-1);
    LLM>>parent->getLLM()(Iu);
  }

  void Object::initz()
  {
    // INITZ initialises the Object state
    q = q0;
    u = u0;
    x = x0;
  }

  string Object::getFullName() const {
    return parent->getFullName() + "." + name;
  }

  void Object::plotParameters() {
    Element::plotParameters();
    // all CoordinateSystem of Object
    parafile << "# Coordinate systems:" << endl;
    for(vector<CoordinateSystem*>::iterator i = port.begin();  i != port.end();  ++i)
      parafile << (**i).getName() << endl;
    // all Contours of Object
    parafile << "# Contours:" << endl;
    for(vector<Contour*>::iterator i = contour.begin();  i != contour.end();  ++i)
      parafile << (**i).getName() << endl;

    parafile << "# q0:" << endl;
    parafile << q0 << endl;
    parafile << "# u0:" << endl;
    parafile << u0 << endl;

    for(vector<CoordinateSystem*>::iterator i = port.begin();  i != port.end();  ++i) 
      (**i).plotParameters();
    for(vector<Contour*>::iterator i = contour.begin();  i != contour.end();  ++i) 
      (**i).plotParameters();
  }

  void Object::load(ifstream& inputfile) {
    Element::load(inputfile);
    cout << name << endl;
    char dummy[10000];
    inputfile.getline(dummy,10000); // # CoSy
    inputfile.getline(dummy,10000); // O
    inputfile.getline(dummy,10000); // # Contours
    inputfile.getline(dummy,10000); // # q0
    inputfile >> q0; // # q0
    inputfile.getline(dummy,10000); // # u0
    inputfile >> u0; // # q0
    cout << q0 << endl;
    cout << u0 << endl;
    cout << dummy << endl;
//    name = dummy;
//    inputfile.getline(dummy,10000);
//    inputfile.getline(dummy,10000);
  }

  void Object::plot(double t, double dt) {
    Element::plot(t);

    for(unsigned int j=0; j<port.size(); j++) {
      port[j]->plot(t,dt);
    }    
    for(unsigned int j=0; j<contour.size(); j++) {
      contour[j]->plot(t,dt);
    }    

    if(plotLevel>1) {
      for(int i=0; i<qSize; ++i)
	plotfile<<" "<<q(i);
      for(int i=0; i<uSize; ++i)
	plotfile<<" "<<u(i);
      for(int i=0; i<xSize; ++i)
	plotfile<<" "<<x(i);
      if(plotLevel>2) {
	for(int i=0; i<qSize; ++i)
	  plotfile<<" "<<qd(i)/dt;
	for(int i=0; i<uSize; ++i)
	  plotfile<<" "<<ud(i)/dt;
	for(int i=0; i<xSize; ++i)
	  plotfile<<" "<<xd(i)/dt;
	for(int i=0; i<uSize; ++i)
	  plotfile<<" "<<h(i);
	for(int i=0; i<uSize; ++i)
	  plotfile<<" "<<r(i)/dt;
      }
    }
  }

  void Object::initPlotFiles() {
    Element::initPlotFiles();

    for(unsigned int j=0; j<port.size(); j++) {
      port[j]->initPlotFiles();
    }    
    for(unsigned int j=0; j<contour.size(); j++) {
      contour[j]->initPlotFiles();
    }    

    if(plotLevel>1) {
      for(int i=0; i<qSize; ++i)
	plotfile <<"# "<< plotNr++ << ": q(" << i << ")" << endl;

      for(int i=0; i<uSize; ++i)
	plotfile <<"# "<< plotNr++ <<": u("<<i<<")" << endl;

      for(int i=0; i<xSize; ++i)
	plotfile <<"# "<< plotNr++ << ": x(" << i << ")" << endl;

      if(plotLevel>2) {
	for(int i=0; i<qSize; ++i)
	  plotfile <<"# "<< plotNr++ << ": qd(" << i << ")" << endl;

	for(int i=0; i<uSize; ++i)
	  plotfile <<"# "<< plotNr++ <<": ud("<<i<<")" << endl;

	for(int i=0; i<xSize; ++i)
	  plotfile <<"# "<< plotNr++ <<": xd("<<i<<")" << endl;

	for(int i=0; i<uSize; ++i)
	  plotfile <<"# "<< plotNr++ <<": h("<<i<<")" << endl;

	for(int i=0; i<uSize; ++i)
	  plotfile <<"# "<< plotNr++ <<": r("<<i<<")" << endl;

	if (plotLevel>3) {									//HR 04.01.2007
	  for(unsigned int j=0; j<port.size(); j++) {
	    for(int i=0; i<3; i++)
	      plotfile<< "# " << plotNr++ <<": WrOP ("<<port[j]->getName()<<")" << endl;
	  }    
	}   
      }
    }
  }

  int Object::portIndex(const CoordinateSystem *port_) const {
    for(unsigned int i=0; i<port.size(); i++) {
      if(port_==port[i])
	return i;
    }
    return -1;
  }

  void Object::addContour(Contour* contour_) {
    if(getContour(contour_->getName(),false)) { //Contourname exists already
      cout << "Error: The Object " << name << " can only comprise one Contour by the name " <<  contour_->getName() << "!" << endl;
      assert(getContour(contour_->getName(),false)==NULL);
    }
    //contour_->setFullName(getFullName()+"."+contour_->getFullName());
    contour.push_back(contour_);
    contour_->setObject(this);
  }

  void Object::addCoordinateSystem(CoordinateSystem* port_) {
    if(getCoordinateSystem(port_->getName(),false)) { //Contourname exists already
      cout << "Error: The Object " << name << " can only comprise one CoordinateSystem by the name " <<  port_->getName() << "!" << endl;
      assert(getCoordinateSystem(port_->getName(),false)==NULL);
    }
    //port_->setFullName(getFullName()+"."+port_->getFullName());
    port.push_back(port_);
    port_->setObject(this);
  }

  CoordinateSystem* Object::getCoordinateSystem(const string &name, bool check) {
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

  Contour* Object::getContour(const string &name, bool check) {
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

  void Object::init() 
  {  
    Iu = Index(uInd,uInd+uSize-1);
    Ix = Index(xInd,xInd+xSize-1);
    Ih = Index(hInd,hInd+hSize-1);

    for(vector<Contour*>::iterator i=contour.begin(); i!=contour.end(); i++) (*i)->init();
  }

  void Object::updatedq(double t, double dt) {
    qd = T*u*dt;
  }

  void Object::updatedu(double t, double dt) {

    ud = slvLLFac(LLM, h*dt+r);
  }

  void Object::updatezd(double t) {

    qd = T*u;
    ud =  slvLLFac(LLM, h+r);
  }

  void Object::facLLM() {
    // FACLLM computes Cholesky decomposition of the mass matrix
    LLM = facLL(M); 
  }

  void Object::updateJh(double t) {
    // static const double eps = epsroot();
    // Vec hOld = geth().copy();
    // int pos = getqInd();
    // for(int j=0;j<q.size();j++) {
    //   Vec Jhcol = parent->getJh().col(pos+j);
    //   double qSave = q(j);
    //   q(j) += eps;
    //   updateKinematics(t);
    //   for(unsigned int i=0; i<linkSingleValued.size(); i++) {
    //     linkSingleValued[i]->updateStage1(t); 
    //     linkSingleValued[i]->updateStage2(t); 
    //   }
    //   updateh(t);
    //   Jhcol(Iu) += (geth()-hOld)/eps;
    //   for(unsigned int i=0; i<linkSingleValuedCoordinateSysteCoordinateSystema.size(); i++) {
    //     LinkCoordinateSystem* l = linkSingleValuedCoordinateSystemData[i].link;
    //     vector<CoordinateSystem*> ports = l->getCoordinateSystems();
    //     for(unsigned int b=0; b<ports.size(); b++) {
    //       Object *obj = ports[b]->getObject()->getResponsible();
    //       if(obj != parent && obj != this) { // Achtung: unser MBS ist auch ein Object, hat aber selber (als eigenstängiges System) keine Freiheiten sondern ruht inertial
    //         Vec hOld = obj->geth().copy();
    //         obj->updateh(t);
    //         Jhcol(obj->Iu) += (obj->geth()-hOld)/eps;
    //         obj->geth() = hOld;
    //       }
    //     }
    //   }
    //   for(unsigned int i=0; i<linkSingleValuedContourData.size(); i++) {
    //     LinkContour* l = linkSingleValuedContourData[i].link;
    //     vector<Contour*> contours = l->getContours();
    //     for(unsigned int b=0; b<contours.size(); b++) {
    //       Object *obj = contours[b]->getObject()->getResponsible();
    //       if(obj != parent && obj != this) { // Achtung: unser MBS ist auch ein Object, hat aber selber (als eigenstängiges System) keine Freiheiten sondern ruht inertial
    //         Vec hOld = obj->geth().copy();
    //         obj->updateh(t);
    //         Jhcol(obj->Iu) += (obj->geth()-hOld)/eps;
    //         obj->geth() = hOld;
    //       }
    //     }
    //   }
    //   q(j) = qSave;
    // }

    // pos = getuInd();
    // for(int j=0;j<u.size();j++) {
    //   Vec Jhcol = parent->getJh().col(parent->getqSize()+pos+j);
    //   double uSave = u(j);
    //   u(j) += eps;
    //   updateKinematics(t);
    //   for(unsigned int i=0; i<linkSingleValued.size(); i++) {
    //     linkSingleValued[i]->updateStage1(t); linkSingleValued[i]->updateStage2(t); 
    //   }
    //   updateh(t);
    //   Jhcol(Iu) += (geth()-hOld)/eps;
    //   for(unsigned int i=0; i<linkSingleValuedCoordinateSystemData.size(); i++) {
    //     LinkCoordinateSystem* l = linkSingleValuedCoordinateSystemData[i].link;
    //     vector<CoordinateSystem*> ports = l->getCoordinateSystems();
    //     for(unsigned int b=0; b<ports.size(); b++) {
    //       Object *obj = ports[b]->getObject()->getResponsible();
    //       if(obj != parent && obj != this) { // Achtung: unser MBS ist auch ein Object, hat aber selber (als eigenstängiges System) keine Freiheiten sondern ruht inertial
    //         Vec hOld = obj->geth().copy();
    //         obj->updateh(t);
    //         Jhcol(obj->Iu) += (obj->geth()-hOld)/eps;
    //         obj->geth() = hOld;
    //       }
    //     }
    //   }
    //   for(unsigned int i=0; i<linkSingleValuedContourData.size(); i++) {
    //     LinkContour* l = linkSingleValuedContourData[i].link;
    //     vector<Contour*> contours = l->getContours();
    //     for(unsigned int b=0; b<contours.size(); b++) {
    //       Object *obj = contours[b]->getObject()->getResponsible();
    //       if(obj != parent && obj != this) { // Achtung: unser MBS ist auch ein Object, hat aber selber (als eigenstängiges System) keine Freiheiten sondern ruht inertial
    //         Vec hOld = obj->geth().copy();
    //         obj->updateh(t);
    //         Jhcol(obj->Iu) += (obj->geth()-hOld)/eps;
    //         obj->geth() = hOld;
    //       }
    //     }
    //   }
    //   u(j) = uSave;
    // }
    // updateKinematics(t);
    // for(unsigned int i=0; i<linkSingleValued.size(); i++) {
    //   linkSingleValued[i]->updateStage1(t); 
    //   linkSingleValued[i]->updateStage2(t); 
    // }
    // h=hOld;
  }


  double Object::computeKineticEnergy() {
    return 0.5*trans(u)*M*u;
  }

}
