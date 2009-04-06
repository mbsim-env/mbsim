/* Copyright (C) 2004-2009 MBSim Development Team
 *
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
#include "mbsim/group.h"
#include "mbsim/object.h"
#include "mbsim/link.h"
#include "mbsim/order_one_dynamics.h"
#include "mbsim/frame.h"
#include "mbsim/contour.h"
#include "mbsim/class_factory.h"
#include "mbsim/dynamic_system_solver.h"
#include "hdf5serie/simpleattribute.h"

#include "compatibility_classes/tree_rigid.h"
#include "compatibility_classes/body_rigid.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Group::Group(const string &name) : DynamicSystem(name) {}

  Group::~Group() {}

  void Group::facLLM() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (*i)->facLLM();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->facLLM();
  }

  void Group::updateKinematics(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateKinematics(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateKinematics(t);
  }

  void Group::updateJacobians(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateJacobians(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateJacobians(t);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (*i)->updateJacobians(t);
  }

  void Group::updatedu(double t, double dt) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updatedu(t,dt);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (*i)->updatedu(t,dt);
  }

  void Group::updatezd(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updatezd(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updatezd(t);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (*i)->updatexd(t);

    for(vector<OrderOneDynamics*>::iterator i = orderOneDynamics.begin(); i!= orderOneDynamics.end(); ++i) 
      (*i)->updatexd(t);
  }

  void Group::updateSecondJacobians(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateSecondJacobians(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateSecondJacobians(t);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (*i)->updateJacobians(t);
  }

  void Group::load(const string &path, ifstream& inputfile) {
    Element::load(path, inputfile);
    string dummy;

    string basename = path + "/" + getFullName() + ".";

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
        addFrame(new Frame("NoName"));
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

    getline(inputfile,dummy); // # q0
    inputfile >> q0; // # q0
    getline(inputfile,dummy); // Rest of line
    getline(inputfile,dummy); // Newline

    getline(inputfile,dummy); // # u0
    inputfile >> u0; // # q0
    getline(inputfile,dummy); // Rest of line
    getline(inputfile,dummy); // Newline

    getline(inputfile,dummy); // # DynamicSystems
    no=getNumberOfElements(inputfile);
    for(unsigned int i=0; i<no; i++) {
      getline(inputfile,dummy); // # DynamicSystems
      string newname = basename + dummy + ".mdl";
      ifstream newinputfile(newname.c_str(), ios::binary);
      getline(newinputfile,dummy);
      getline(newinputfile,dummy);
      ClassFactory cf;
      DynamicSystem * newdynamicsystem = cf.getDynamicSystem(dummy);
      //addDynamicSystem(newdynamicsystem); TODO
      newinputfile.seekg(0,ios::beg);
      newdynamicsystem->setDynamicSystemSolver(ds);
      newdynamicsystem->load(path,newinputfile);
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
      newobject->setDynamicSystemSolver(ds);
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
      newlink->setDynamicSystemSolver(ds);
      newlink->load(path,newinputfile);
      newinputfile.close();
    }
    getline(inputfile,dummy); // newline

    getline(inputfile,dummy); // # order one dynamics
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

    if(ds != this) {
      getline(inputfile,dummy); // # Coordinate system for kinematics
      getline(inputfile,dummy); // Coordinate system for kinematics
      //setFrameForKinematics(getFrame(dummy));
      getline(inputfile,dummy); // newline

      getline(inputfile,dummy); // # Frame of reference
      getline(inputfile,dummy); // Coordinate system for kinematics
      //setFrameOfReference(getDynamicSystemSolver()->findFrame(dummy));
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

  void Group::save(const string &path, ofstream& outputfile) {
    Element::save(path,outputfile);

    // all Frame of Object
    outputfile << "# Coordinate systems:" << endl;
    for(vector<Frame*>::iterator i = port.begin();  i != port.end();  ++i) {
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

    // all DynamicSystems of DynamicSystems
    outputfile << "# DynamicSystems:" << endl;
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin();  i != dynamicsystem.end();  ++i) {
      outputfile << (**i).getName() << endl;
      string newname = path + "/" + (**i).getFullName() + ".mdl";
      ofstream newoutputfile(newname.c_str(), ios::binary);
      (**i).save(path,newoutputfile);
      newoutputfile.close();
    }
    outputfile << endl;

    // all Objects of DynamicSystems
    outputfile << "# Objects:" << endl;
    for(vector<Object*>::iterator i = object.begin();  i != object.end();  ++i) {
      outputfile << (**i).getName() << endl;
      string newname = path + "/" + (**i).getFullName() + ".mdl";
      ofstream newoutputfile(newname.c_str(), ios::binary);
      (**i).save(path,newoutputfile);
      newoutputfile.close();
    }
    outputfile << endl;

    // all Links of DynamicSystems
    outputfile << "# Links:" << endl;
    for(vector<Link*>::iterator i = link.begin();  i != link.end();  ++i) {
      outputfile << (**i).getName() << endl;
      string newname = path + "/" + (**i).getFullName() + ".mdl";
      ofstream newoutputfile(newname.c_str(), ios::binary);
      (**i).save(path,newoutputfile);
      newoutputfile.close();
    }
    outputfile << endl;

    // all order one dynamics of DynamicSystems
    outputfile << "# order one dynamics:" << endl;
    for(vector<OrderOneDynamics*>::iterator i = orderOneDynamics.begin();  i != orderOneDynamics.end();  ++i) {
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
  }

  void Group::addDynamicSystem(DynamicSystem *sys, const Vec &RrRS, const SqrMat &ARS, const Frame* refFrame) {
    DynamicSystem::addDynamicSystem(sys);

    int i = 0;
    if(refFrame)
      i = portIndex(refFrame);

    IrOS.push_back(IrOK[i] + AIK[i]*RrRS);
    AIS.push_back(AIK[i]*ARS);
  }
}

