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
#include "class_factory.h"
#include "multi_body_system.h"

#include "compatibility_classes/tree_rigid.h"
#include "compatibility_classes/body_rigid.h"

namespace MBSim {

  Subsystem::Subsystem(const string &name) : Element(name), qSize(0), qInd(0), uSize(0), uInd(0), xSize(0), xInd(0), hSize(0), hInd(0), gSize(0), gInd(0), gdSize(0), gdInd(0), laSize(0), laInd(0), rFactorSize(0), rFactorInd(0), svSize(0), svInd(0), q0(qSize), u0(uSize), x0(xSize) {

    CoordinateSystem *cosy = new CoordinateSystem("I");
    addCoordinateSystem(cosy);

    IrOK.push_back(Vec(3));
    AIK.push_back(SqrMat(3,EYE));

    cosy->setPosition(Vec(3));
    cosy->setOrientation(SqrMat(3,EYE));
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
      subsystem[i]->getCoordinateSystem("I")->setPosition(port[0]->getPosition() + port[0]->getOrientation()*IrOS[i]);
      subsystem[i]->getCoordinateSystem("I")->setOrientation(port[0]->getOrientation()*AIS[i]);
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

  void Subsystem::closePlotFiles() {
    Element::closePlotFiles();
    for(unsigned i=0; i<subsystem.size(); i++)
      subsystem[i]->closePlotFiles();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->closePlotFiles();
    for(unsigned i=0; i<link.size(); i++)
      link[i]->closePlotFiles();
    for(unsigned i=0; i<EDI.size(); i++)
      EDI[i]->closePlotFiles();
  }

  void Subsystem::initPlotFiles() {
    Element::initPlotFiles();
    for(unsigned i=0; i<subsystem.size(); i++)
      subsystem[i]->initPlotFiles();
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
    Element::plot(t,dt);
    for(unsigned i=0; i<subsystem.size(); i++)
      subsystem[i]->plot(t,dt);
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

  void Subsystem::save(const string &path, ofstream& outputfile) {

    Element::save(path,outputfile);

    // all CoordinateSystem of Object
    outputfile << "# Coordinate systems:" << endl;
    for(vector<CoordinateSystem*>::iterator i = port.begin();  i != port.end();  ++i) {
      outputfile << (**i).getName() << endl;
      string newname = path + "/" + (**i).getFullName() + ".mdl";
      ofstream newoutputfile(newname.c_str(), ios::binary);
      (**i).save(path,newoutputfile);
      newoutputfile.close();
    }
    outputfile << endl;

    // all Contours of Object
    outputfile << "# Contours:" << endl;
    for(vector<Contour*>::iterator i = contour.begin();  i != contour.end();  ++i) {
      outputfile << (**i).getName() << endl;
      string newname = path + "/" + (**i).getFullName() + ".mdl";
      ofstream newoutputfile(newname.c_str(), ios::binary);
      (**i).save(path,newoutputfile);
      newoutputfile.close();
    }
    outputfile << endl;

    outputfile << "# q0:" << endl;
    outputfile << q0 << endl << endl;
    outputfile << "# u0:" << endl;
    outputfile << u0 << endl << endl;

    // all Subsystems of Subsystems
    outputfile << "# Subsystems:" << endl;
    for(vector<Subsystem*>::iterator i = subsystem.begin();  i != subsystem.end();  ++i) {
      outputfile << (**i).getName() << endl;
      string newname = path + "/" + (**i).getFullName() + ".mdl";
      ofstream newoutputfile(newname.c_str(), ios::binary);
      (**i).save(path,newoutputfile);
      newoutputfile.close();
    }
    outputfile << endl;

    // all Objects of Subsystems

    outputfile << "# Objects:" << endl;
    for(vector<Object*>::iterator i = object.begin();  i != object.end();  ++i) {
      outputfile << (**i).getName() << endl;
      string newname = path + "/" + (**i).getFullName() + ".mdl";
      ofstream newoutputfile(newname.c_str(), ios::binary);
      (**i).save(path,newoutputfile);
      newoutputfile.close();
    }
    outputfile << endl;

    // all Links of Subsystems
    outputfile << "# Links:" << endl;
    for(vector<Link*>::iterator i = link.begin();  i != link.end();  ++i) {
      outputfile << (**i).getName() << endl;
      string newname = path + "/" + (**i).getFullName() + ".mdl";
      ofstream newoutputfile(newname.c_str(), ios::binary);
      (**i).save(path,newoutputfile);
      newoutputfile.close();
    }
    outputfile << endl;

    // all EDIs of Subsystems
    outputfile << "# EDIs:" << endl;
    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin();  i != EDI.end();  ++i) {
      outputfile << (**i).getName() << endl;
      string newname = path + "/" + (**i).getFullName() + ".mdl";
      ofstream newoutputfile(newname.c_str(), ios::binary);
      (**i).save(path,newoutputfile);
      newoutputfile.close();
    }

    outputfile << endl;

    for(unsigned int i=1; i<port.size(); i++) {
      outputfile << "# Translation of coordinate system " << port[i]->getName() <<":" << endl;
      outputfile << IrOK[i] << endl;
      outputfile << endl;
      outputfile << "# Rotation of coordinate system "  << port[i]->getName() <<":" << endl;
      outputfile << AIK[i] << endl;
      outputfile << endl;
    }

    for(unsigned int i=0; i<contour.size(); i++) {
      outputfile << "# Translation of contour " << contour[i]->getName() <<":" << endl;
      outputfile << IrOC[i] << endl;
      outputfile << endl;
      outputfile << "# Rotation of contour " << contour[i]->getName() <<":" << endl;
      outputfile << AIC[i] << endl;
      outputfile << endl;
    }

    //   if(mbs != this) {
    //     outputfile << "# Reference coordinate system:" << endl;
    //     outputfile << port[iRef]->getName() << endl;
    //     outputfile << endl;

    //     outputfile << "# Parent coordinate system:" << endl;
    //     outputfile << portParent->getFullName() << endl;
    //     outputfile << endl;

    //     outputfile << "# Translation:" << endl;
    //     outputfile << PrPK << endl;
    //     outputfile << endl;

    //     outputfile << "# Rotation:" << endl;
    //     outputfile << APK << endl;
    //     outputfile << endl;
    //   }
  }

  void Subsystem::load(const string &path, ifstream& inputfile) {
    Element::load(path, inputfile);
    string dummy;

    string basename = path + "/" + getFullName() + ".";

    cout << name << endl;
    cout << fullName << endl;
    getline(inputfile,dummy); // # CoSy
    unsigned int no=getNumberOfElements(inputfile);
    for(unsigned int i=0; i<no; i++) {
      getline(inputfile,dummy); // CoSy
      string newname = basename + dummy + ".mdl";
      ifstream newinputfile(newname.c_str(), ios::binary);
      getline(newinputfile,dummy);
      getline(newinputfile,dummy);
      newinputfile.seekg(0,ios::beg);
      if(i>=port.size())
	addCoordinateSystem(new CoordinateSystem("NoName"));
      port[i]->load(path, newinputfile);
      newinputfile.close();
    }
    getline(inputfile,dummy); // # newline

    getline(inputfile,dummy); // # Contour
    no=getNumberOfElements(inputfile);
    for(unsigned int i=0; i<no; i++) {
      getline(inputfile,dummy); // contour
      string newname = basename + dummy + ".mdl";
      ifstream newinputfile(newname.c_str(), ios::binary);
      getline(newinputfile,dummy);
      getline(newinputfile,dummy);
      newinputfile.seekg(0,ios::beg);
      ClassFactory cf;
      if(i>=contour.size())
	addContour(cf.getContour(dummy));
      contour[i]->load(path, newinputfile);
      newinputfile.close();
    }
    getline(inputfile,dummy); // newline

    cout << dummy << endl;
    getline(inputfile,dummy); // # q0
    cout << dummy << endl;
    inputfile >> q0; // # q0
    getline(inputfile,dummy); // Rest of line
    getline(inputfile,dummy); // Newline

    getline(inputfile,dummy); // # u0
    inputfile >> u0; // # q0
    getline(inputfile,dummy); // Rest of line
    getline(inputfile,dummy); // Newline

    getline(inputfile,dummy); // # Subsystems
    no=getNumberOfElements(inputfile);
    for(unsigned int i=0; i<no; i++) {
      getline(inputfile,dummy); // # Subsystems
      string newname = basename + dummy + ".mdl";
      ifstream newinputfile(newname.c_str(), ios::binary);
      getline(newinputfile,dummy);
      getline(newinputfile,dummy);
      ClassFactory cf;
      Subsystem * newobject = cf.getSubsystem(dummy);
      //addSubsystem(newobject);
      newinputfile.seekg(0,ios::beg);
      newobject->load(path,newinputfile);
      newinputfile.close();
    }
    getline(inputfile,dummy); // newline

    getline(inputfile,dummy); // # Objects
    no=getNumberOfElements(inputfile);
    for(unsigned int i=0; i<no; i++) {
      getline(inputfile,dummy); // # Objects
      string newname = basename + dummy + ".mdl";
      ifstream newinputfile(newname.c_str(), ios::binary);
      getline(newinputfile,dummy);
      getline(newinputfile,dummy);
      ClassFactory cf;
      Object * newobject = cf.getObject(dummy);
      addObject(newobject);
      newinputfile.seekg(0,ios::beg);
      newobject->load(path,newinputfile);
      newinputfile.close();
    }
    getline(inputfile,dummy); // newline

    getline(inputfile,dummy); // # Links
    no=getNumberOfElements(inputfile);
    for(unsigned int i=0; i<no; i++) {
      getline(inputfile,dummy); // # Links
      string newname = basename + dummy + ".mdl";
      ifstream newinputfile(newname.c_str(), ios::binary);
      getline(newinputfile,dummy);
      getline(newinputfile,dummy);
      ClassFactory cf;
      Link * newlink = cf.getLink(dummy);
      addLink(newlink);
      newinputfile.seekg(0,ios::beg);
      newlink->load(path,newinputfile);
      newinputfile.close();
    }
    getline(inputfile,dummy); // newline

    getline(inputfile,dummy); // # EDIs
    getline(inputfile,dummy); // newline

    for(unsigned int i=1; i<port.size(); i++) {
      IrOK.push_back(Vec(3));
      AIK.push_back(SqrMat(3));
      getline(inputfile,dummy); // # Translation cosy 
      inputfile >> IrOK[i];
      getline(inputfile,dummy); // Rest of line
      getline(inputfile,dummy); // newline
      getline(inputfile,dummy); // # Rotation cosy
      inputfile >> AIK[i];
      getline(inputfile,dummy); // Rest of line
      getline(inputfile,dummy); // newline
    }

    for(unsigned int i=0; i<contour.size(); i++) {
      IrOC.push_back(Vec(3));
      AIC.push_back(SqrMat(3));
      getline(inputfile,dummy); // # Translation contour 
      inputfile >> IrOC[i];
      getline(inputfile,dummy); // Rest of line
      getline(inputfile,dummy); // newline
      getline(inputfile,dummy); // # Rotation contour
      inputfile >> AIC[i];
      getline(inputfile,dummy); // Rest of line
      getline(inputfile,dummy); // newline
    }

    if(mbs != this) {
      getline(inputfile,dummy); // # Coordinate system for kinematics
      getline(inputfile,dummy); // Coordinate system for kinematics
      //setCoordinateSystemForKinematics(getCoordinateSystem(dummy));
      getline(inputfile,dummy); // newline

      getline(inputfile,dummy); // # Frame of reference
      getline(inputfile,dummy); // Coordinate system for kinematics
      //setFrameOfReference(getMultiBodySystem()->findCoordinateSystem(dummy));
      getline(inputfile,dummy); // newline

      getline(inputfile,dummy); // # Translation 
      Vec r;
      inputfile >> r;
      getline(inputfile,dummy); // Rest of line
      getline(inputfile,dummy); // newline
      //setTranslation(r);

      getline(inputfile,dummy); // # Rotation
      SqrMat A;
      inputfile >> A;
      getline(inputfile,dummy); // Rest of line
      getline(inputfile,dummy); // newline
      //setRotation(A);
    }

  }

  void Subsystem::addObject(Object *obj) {
    // ADDOBJECT adds an object
    if(getObject(obj->getName(),false)) {
      cout << "Error: The Subsystem " << name << " can only comprise one Object by the name " <<  obj->getName() << "!" << endl;
      assert(getObject(obj->getName(),false) == NULL); 
    }
    //obj->setFullName(getFullName()+"."+obj->getFullName());
    object.push_back(obj);
    //obj->setMbs(this);
    //obj->setParent(this);
  }

  void Subsystem::addLink(Link *lnk) {
    if(getLink(lnk->getName(),false)) {
      cout << "Error: The Subsystem " << name << " can only comprise one Link by the name " <<  lnk->getName() << "!" << endl;
      assert(getLink(lnk->getName(),false) == NULL);
    }

    link.push_back(lnk);
    if(lnk->isSetValued()) {
      linkSetValued.push_back(lnk);
      linkSetValuedActive.push_back(lnk);
    }
    else 
      linkSingleValued.push_back(lnk);

    //lnk->setParent(this);
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


  void Subsystem::updateKinematics(double t) {
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateKinematics(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateKinematics(t);
  }

  void Subsystem::updateg(double t) {
    //if(!HSLink.empty()) {
    //  linkSingleValued.erase(linkSingleValued.begin()+nHSLinkSingleValuedFixed,linkSingleValued.end());
    //  linkSetValued.erase(linkSetValued.begin()+nHSLinkSetValuedFixed,linkSetValued.end());
    //  for(vector<HitSphereLink*>::iterator iHS = HSLink.begin(); iHS != HSLink.end(); ++iHS) (*iHS)->checkActive();
    //}

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

    u >> uParent(uInd,uInd+uSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updateuRef(u);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateuRef(u);
  }

  void Subsystem::updateudRef(const Vec &udParent) {

    ud >> udParent(uInd,uInd+uSize-1);

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

  void Subsystem::updatehRef(const Vec &hParent) {

    h >> hParent(hInd,hInd+hSize-1);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (**i).updatehRef(h);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatehRef(h);

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updatehRef(h);
  }

  void Subsystem::updaterRef(const Vec &rParent) {

    r >> rParent(hInd,hInd+hSize-1);

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

  void Subsystem::updateMRef(const SymMat& MParent) {

    M >> MParent(Index(hInd,hInd+hSize-1));

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateMRef(M);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateMRef(M);
  }

  void Subsystem::updateLLMRef(const SymMat& LLMParent) {

    LLM >> LLMParent(Index(hInd,hInd+hSize-1));

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateLLMRef(LLM);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateLLMRef(LLM);
  }

  void Subsystem::updateTRef(const Mat& TParent) {

    T >> TParent(Index(qInd,qInd+qSize-1),Index(uInd,uInd+uSize-1));

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

  void Subsystem::updateVRef(const Mat &VParent) {

    V.resize() >> VParent(Index(uInd,uInd+uSize-1),Index(laInd,laInd+laSize-1));

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateVRef(V);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updateVRef(V);
  }

  void Subsystem::updateWRef(const Mat &WParent) {
    W.resize() >> WParent(Index(uInd,uInd+uSize-1),Index(laInd,laInd+laSize-1));

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updateWRef(W);

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).updateWRef(W);
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

  int Subsystem::portIndex(const CoordinateSystem *port_) const {
    for(unsigned int i=0; i<port.size(); i++) {
      if(port_==port[i])
	return i;
    }
    return -1;
  }

  CoordinateSystem* Subsystem::getCoordinateSystem(const string &name, bool check) {
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

  void Subsystem::addSubsystem(Subsystem *sys, const Vec &RrRS, const SqrMat &ARS, const CoordinateSystem* refCoordinateSystem) {
    // ADDOBJECT adds an subsystem
    if(getSubsystem(sys->getName(),false)) {
      cout << "Error: The Subsystem " << name << " can only comprise one Object by the name " <<  sys->getName() << "!" << endl;
      assert(getSubsystem(sys->getName(),false) == NULL); 
    }
    subsystem.push_back(sys);

    int i = 0;
    if(refCoordinateSystem)
      i = portIndex(refCoordinateSystem);

    IrOS.push_back(IrOK[i] + AIK[i]*RrRS);
    AIS.push_back(AIK[i]*ARS);
  }

  void Subsystem::addCoordinateSystem(CoordinateSystem* cosy) {

    if(getCoordinateSystem(cosy->getName(),false)) { //Contourname exists already
      cout << "Error: The Subsystem " << name << " can only comprise one CoordinateSystem by the name " <<  cosy->getName() << "!" << endl;
      assert(getCoordinateSystem(cosy->getName(),false)==NULL);
    }
    port.push_back(cosy);
  }

  void Subsystem::addCoordinateSystem(CoordinateSystem* cosy, const Vec &RrRK, const SqrMat &ARK, const CoordinateSystem* refCoordinateSystem) {

    addCoordinateSystem(cosy);

    int i = 0;
    if(refCoordinateSystem)
      i = portIndex(refCoordinateSystem);

    IrOK.push_back(IrOK[i] + AIK[i]*RrRK);
    AIK.push_back(AIK[i]*ARK);
  }

  void Subsystem::addCoordinateSystem(const string &str, const Vec &SrSK, const SqrMat &ASK, const CoordinateSystem* refCoordinateSystem) {
    addCoordinateSystem(new CoordinateSystem(str),SrSK,ASK,refCoordinateSystem);
  }

  void Subsystem::addContour(Contour* contour_) {

    if(getContour(contour_->getName(),false)) { //Contourname exists already
      cout << "Error: The Subsystem " << name << " can only comprise one Contour by the name " <<  contour_->getName() << "!" << endl;
      assert(getContour(contour_->getName(),false)==NULL);
    }
    contour.push_back(contour_);
  }

  void Subsystem::addContour(Contour* contour, const Vec &RrRC, const SqrMat &ARC, const CoordinateSystem* refCoordinateSystem) {

    addContour(contour);

    int i = 0;
    if(refCoordinateSystem)
      i = portIndex(refCoordinateSystem);

    IrOC.push_back(IrOK[i] + AIK[i]*RrRC);
    AIC.push_back(AIK[i]*ARC);
  }

  void Subsystem::calchSize() {

    hSize = uSize;
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
      (*i)->sethSize((*i)->getuSize());
      (*i)->sethInd((*i)->getuInd());
      (*i)->calchSize();
    }
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->sethSize((*i)->getuSize());
      (*i)->sethInd((*i)->getuInd());
      (*i)->calchSize();
    }
  }

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

  void Subsystem::calcuSize() {
    uSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
      (*i)->calcuSize();
      (*i)->setuInd(uSize);
      uSize += (*i)->getuSize();
    }
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->calcuSize();
      (*i)->setuInd(uSize);
      uSize += (*i)->getuSize();
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

  void Subsystem::calcgSize() {
    gSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
      (*i)->calcgSize();
      (*i)->setgInd(gSize);
      gSize += (*i)->getgSize();
    }

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (*i)->calcgSize();
      (*i)->setgInd(gSize);
      gSize += (*i)->getgSize();
    }
  }

  void Subsystem::calcgdSize() {
    gdSize = 0;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) {
      (*i)->calcgdSize();
      (*i)->setgdInd(gdSize);
      gdSize += (*i)->getgdSize();
    }

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
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

  //bool Subsystem::activeConstraintsChanged() {

  //  for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
  //    if ((*i)->activeConstraintsChanged())
  //      return true;
  //  
  //  for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
  //    if ((*i)->activeConstraintsChanged())
  //      return true;

  //  return false;
  //}

  bool Subsystem::gActiveChanged() {

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      if ((*i)->gActiveChanged())
	return true;

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      if ((*i)->gActiveChanged())
	return true;

    return false;
  }

  //  bool Subsystem::activeHolonomicConstraintsChanged() {
  //
  //    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
  //      if ((*i)->activeHolonomicConstraintsChanged())
  //	return true;
  //    
  //    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
  //      if ((*i)->activeHolonomicConstraintsChanged())
  //	return true;
  //
  //    return false;
  //  }

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

  void Subsystem::addObject(TreeRigid *tree) {
    tree->setFullName(name+"."+tree->getName());
    tree->setParent(this);
    addSubsystem(tree,Vec(3),SqrMat(3,EYE));
  }
  void Subsystem::addObject(BodyRigid *body) {
    // ADDOBJECT adds an object
    body->setFullName(name+"."+body->getName());
    body->setParent(this);
    if(getObject(body->getName(),false)) {
      cout << "Error: The Subsystem " << name << " can only comprise one Object by the name " <<  body->getName() << "!" << endl;
      assert(getObject(body->getName(),false) == NULL); 
    }
    object.push_back(body);
  }
}
