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
#include "dynamic_system_solver.h"
#include "objectfactory.h"
#include "mainwindow.h"
#include <QDir>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;
  extern bool absolutePath;
  extern QDir mbsDir;

  Solver::Solver() : embed(0,false), name("Solver") {

    embed.setProperty(new EmbedProperty(name));
  }

  Solver::~Solver() {
  }

  void Solver::initializeUsingXML(DOMElement *element) {
  }

  DOMElement* Solver::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(MBSIMINT%getType());
    parent->insertBefore(ele0, NULL);

    return ele0;
  }

  void Solver::initializeUsingXMLEmbed(DOMElement *element) {
    embed.initializeUsingXML(element);
    embed.setActive(true);
  }

  DOMElement* Solver::writeXMLFileEmbed(DOMNode *parent) {
    DOMElement *ele = embed.writeXMLFile(parent);

    //  if(static_cast<const EmbedProperty*>(embed.getProperty())->hasParameterFile()) {
    //    string absFileName =  static_cast<const EmbedProperty*>(embed.getProperty())->getParameterFile();
    //    string relFileName =  mbsDir.relativeFilePath(QString::fromStdString(absFileName)).toStdString();
    //    shared_ptr<DOMDocument> doc=MainWindow::parser->createDocument();
    //    DOMElement *ele1 = D(doc)->createElement(PV%"Parameter");
    //    doc->insertBefore( ele1, NULL );
    //    for(int i=0; i<parameter.size(); i++)
    //      parameter[i]->writeXMLFile(ele1);
    //    string name=absolutePath?(mw->getUniqueTempDir().generic_string()+"/"+relFileName):absFileName;
    //    QFileInfo info(QString::fromStdString(name));
    //    QDir dir;
    //    if(!dir.exists(info.absolutePath()))
    //      dir.mkpath(info.absolutePath());
    //    DOMParser::serialize(doc.get(), (name.length()>4 && name.substr(name.length()-4,4)==".xml")?name:name+".xml");
    //  }
    //  else {
    //    DOMElement *ele1 = D(doc)->createElement(PV%"Parameter");
    //    ele->insertBefore( ele1, NULL );
    //    for(int i=0; i<parameter.size(); i++)
    //      parameter[i]->writeXMLFile(ele1);
    //  }

    if(!static_cast<const EmbedProperty*>(embed.getProperty())->hasFile())
      writeXMLFile(ele);
    else {
      string absFileName =  static_cast<const EmbedProperty*>(embed.getProperty())->getFile();
      string relFileName =  mbsDir.relativeFilePath(QString::fromStdString(absFileName)).toStdString();
      string name=absolutePath?(mw->getUniqueTempDir().generic_string()+"/"+relFileName):absFileName;
      writeXMLFile(name);
    }
    return ele;
  }

  Solver* Solver::readXMLFile(const string &filename) {
    MBSimObjectFactory::initialize();
    shared_ptr<DOMDocument> doc=mw->parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
    Solver *solver=ObjectFactory::getInstance()->createSolver(e);
    if(solver)
      solver->initializeUsingXML(e);
    return solver;
  }

  void Solver::writeXMLFile(const string &name) {
    shared_ptr<DOMDocument> doc=mw->parser->createDocument();
    writeXMLFile(doc.get());
    DOMParser::serialize(doc.get(), (name.length()>4 && name.substr(name.length()-4,4)==".xml")?name:name+".xml");
  }

}
