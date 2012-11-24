/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <config.h>
#include "solver.h"
#include <QtGui/QMenu>
#include <QtGui/QPushButton>
#include "objectfactory.h"
#include "editors.h"
#include <string>
#include "utils.h"
#include <QtGui/QMessageBox>

using namespace std;

TiXmlElement* Element::copiedElement;

void Environment::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"accelerationOfGravity");
  //setAccelerationOfGravity(Element::getVec3(e));
}

TiXmlElement* Environment::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement* ele0 = new TiXmlElement( MBSIMNS"MBSimEnvironment" );

  TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"accelerationOfGravity" );
  //TiXmlText *text;// = new TiXmlText( mat2str(getAccelerationOfGravity()) );
  //ele1->LinkEndChild(text);
  ele0->LinkEndChild( ele1 );
  parent->LinkEndChild(ele0);
  return ele0;
}

Environment::Environment() {}
Environment::~Environment() {}

Environment *Environment::instance=NULL;

Solver::Solver(const QString &str, QTreeWidgetItem *parentItem, int ind) : Group(str, parentItem, ind) {

  setText(1,getType());

  Element::copiedElement = 0;

  properties->addTab("Environment");

  environment = new ExtXMLWidget("Acceleration of gravity",new EnvironmentWidget);
  properties->addToTab("Environment", environment);

  properties->addStretch();
}

QString Solver::getInfo() {
  return Element::getInfo()+
         QString("<hr width=\"10000\"/>")+
         QString("<b>Number of children:</b> %1").arg(childCount());
}


  void Solver::initializeUsingXML(TiXmlElement *element) {
    Group::initializeUsingXML(element);
    TiXmlElement *e;
    // search first Environment element
    e=element->FirstChildElement(MBSIMNS"environments")->FirstChildElement();

    Environment *env;
    while((env=ObjectFactory::getInstance()->getEnvironment(e))) {
      env->initializeUsingXML(e);
      environment->initializeUsingXML(e);
      e=e->NextSiblingElement();
    }

//    e=element->FirstChildElement(MBSIMNS"solverParameters");
//    if (e) {
//      TiXmlElement * ee;
//      ee=e->FirstChildElement(MBSIMNS"constraintSolver");
//      if (ee) {
//        if (ee->FirstChildElement(MBSIMNS"FixedPointTotal"))
//          setConstraintSolver(FixedPointTotal);
//        else if (ee->FirstChildElement(MBSIMNS"FixedPointSingle"))
//          setConstraintSolver(FixedPointSingle);
//        else if (ee->FirstChildElement(MBSIMNS"GaussSeidel"))
//          setConstraintSolver(GaussSeidel);
//        else if (ee->FirstChildElement(MBSIMNS"LinearEquations"))
//          setConstraintSolver(LinearEquations);
//        else if (ee->FirstChildElement(MBSIMNS"RootFinding"))
//          setConstraintSolver(RootFinding);
//      }
//      ee=e->FirstChildElement(MBSIMNS"impactSolver");
//      if (ee) {
//        if (ee->FirstChildElement(MBSIMNS"FixedPointTotal"))
//          setImpactSolver(FixedPointTotal);
//        else if (ee->FirstChildElement(MBSIMNS"FixedPointSingle"))
//          setImpactSolver(FixedPointSingle);
//        else if (ee->FirstChildElement(MBSIMNS"GaussSeidel"))
//          setImpactSolver(GaussSeidel);
//        else if (ee->FirstChildElement(MBSIMNS"LinearEquations"))
//          setImpactSolver(LinearEquations);
//        else if (ee->FirstChildElement(MBSIMNS"RootFinding"))
//          setImpactSolver(RootFinding);
//      }
//      ee=e->FirstChildElement(MBSIMNS"numberOfMaximalIterations");
//      if (ee)
//        setMaxIter(atoi(ee->GetText()));
//      ee=e->FirstChildElement(MBSIMNS"tolerances");
//      if (ee) {
//        TiXmlElement * eee;
//        eee=ee->FirstChildElement(MBSIMNS"projection");
//        if (eee)
//          setProjectionTolerance(getDouble(eee));
//        eee=ee->FirstChildElement(MBSIMNS"gd");
//        if (eee)
//          setgdTol(getDouble(eee));
//        eee=ee->FirstChildElement(MBSIMNS"gdd");
//        if (eee)
//          setgddTol(getDouble(eee));
//        eee=ee->FirstChildElement(MBSIMNS"la");
//        if (eee)
//          setlaTol(getDouble(eee));
//        eee=ee->FirstChildElement(MBSIMNS"La");
//        if (eee)
//          setLaTol(getDouble(eee));
//      }
//    }
  }

TiXmlElement* Solver::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Group::writeXMLFile(parent);
  ele0->SetAttribute("xmlns", "http://mbsim.berlios.de/MBSim");
  ele0->SetAttribute("xmlns:ombv", "http://openmbv.berlios.de/OpenMBV");

  TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"environments" );
  environment->writeXMLFile(ele1);

  ele0->LinkEndChild( ele1 );
  return ele0;
}

Solver* Solver::readXMLFile(const QString &filename, QTreeWidgetItem* parent) {
  MBSimObjectFactory::initialize();
  TiXmlDocument doc;
  bool ret=doc.LoadFile(filename.toAscii().data());
  assert(ret==true);
  TiXml_PostLoadFile(&doc);
  TiXmlElement *e=doc.FirstChildElement();
  TiXml_setLineNrFromProcessingInstruction(e);
  map<string,string> dummy;
  incorporateNamespace(doc.FirstChildElement(), dummy);
  Solver *solver=dynamic_cast<Solver*>(ObjectFactory::getInstance()->createGroup(e, parent, 1));
  solver->initializeUsingXML(doc.FirstChildElement());
  solver->initialize();
  return solver;
}

void Solver::writeXMLFile(const QString &name) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  writeXMLFile(&doc);
  map<string, string> nsprefix=ObjectFactory::getInstance()->getNamespacePrefixMapping();
  unIncorporateNamespace(doc.FirstChildElement(), nsprefix);  
  doc.SaveFile((name+".mbsim.xml").toAscii().data());
}
