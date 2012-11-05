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

#include "element.h"
#include <QtGui/QMenu>
#include <QtGui/QApplication>
#include <QtGui/QGridLayout>
#include <QtGui/QInputDialog>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <cmath>
#include "mainwindow.h"
#include "utils.h"
#include "editors.h"
#include "solver.h"
#include "object.h"
#include "frame.h"
#include "link.h"

using namespace std;


//set<Element*> Element::objects;

Element::Element(const QString &str, QTreeWidgetItem *parentItem, int ind, bool grey) : QTreeWidgetItem(), drawThisPath(true), searchMatched(true), frames(0), contours(0), groups(0), objects(0), links(0), extraDynamics(0) {
  if(ind==-1 || ind>=parentItem->childCount())
    parentItem->addChild(this); // insert as last element
  else
    parentItem->insertChild(ind, this); // insert at position ind
  parentElement = static_cast<Element*>(parentItem->parent());

  setText(0, str);

  properties=new PropertyDialog(this);
  properties->addTab("General");

  if(grey) {
    QColor color;
    color.setRgb(200,200,200);
    QBrush brush(color);
    //setForeground(0,brush);
    //setForeground(1,brush);
    name = new NameEditor(this, properties, Utils::QIconCached("lines.svg"), "Name", false);
  }
  else {
    name = new NameEditor(this, properties, Utils::QIconCached("lines.svg"), "Name");
  }

  contextMenu=new QMenu("Context Menu");

//  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Properties", this);
//  connect(action,SIGNAL(triggered()),properties,SLOT(show()));
//  contextMenu->addAction(action);
//
//  contextMenu->addSeparator();
}

Element::~Element() {
  // delete scene graph
  delete properties;
  //objects.erase(this);
}

QString Element::getPath() {
 return parentElement?(parentElement->getPath()+"."+text(0)):text(0);
}

QString Element::getInfo() {
  return QString("<b>Path:</b> %1<br/>").arg(getPath())+
         QString("<b>Class:</b> <img src=\"%1\" width=\"16\" height=\"16\"/> %2").arg(iconFile.c_str()).arg(metaObject()->className());
}

void Element::updateTextColor() {
  QPalette palette;
  if(drawThisPath) // active
    if(searchMatched)
      setForeground(0, palette.brush(QPalette::Active, QPalette::Text));
    else
      setForeground(0, QBrush(QColor(255,0,0)));
  else // inactive
    if(searchMatched)
      setForeground(0, palette.brush(QPalette::Disabled, QPalette::Text));
    else
      setForeground(0, QBrush(QColor(128,0,0)));
}

void Element::saveAs() {
  file=QFileDialog::getSaveFileName(0, "XML model files",  QString("./")+getName()+getFileExtension(), QString("hdf5 Files (*)")+getFileExtension()+")");
  if(file!="") {
    if(file.contains(getFileExtension()))
      file.chop(getFileExtension().size());
    writeXMLFile(file);
    actionSave->setDisabled(false);
  }
}

void Element::save() {
  if(file.contains(getFileExtension()))
    file.chop(getFileExtension().size());
  writeXMLFile(file);
}

void Element::copy() {
  delete copiedElement;
  copiedElement = new TiXmlElement("Node");
  writeXMLFile(copiedElement);
  map<string, string> nsprefix;
  unIncorporateNamespace(copiedElement->FirstChildElement(), nsprefix);  
  ((Solver*)treeWidget()->topLevelItem(0))->setActionPasteDisabled(false);
}

void Element::writeXMLFile(const QString &name) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  writeXMLFile(&doc);
  map<string, string> nsprefix;
  unIncorporateNamespace(doc.FirstChildElement(), nsprefix);  
  doc.SaveFile((name+".xml").toAscii().data());
}

void Element::update() {
  properties->update();
  if(getContainerGroup()) {
    for(int i=0; i<getContainerGroup()->childCount(); i++)
      getGroup(i)->update();
  }
  if(getContainerObject()) {
    for(int i=0; i<getContainerObject()->childCount(); i++)
      getObject(i)->update();
  }
  if(getContainerFrame()) {
    for(int i=0; i<getContainerFrame()->childCount(); i++)
      getFrame(i)->update();
  }
  if(getContainerLink()) {
    for(int i=0; i<getContainerLink()->childCount(); i++)
      getLink(i)->update();
  }
}

void Element::initialize() {
  properties->initialize();
  if(getContainerGroup()) {
    for(int i=0; i<getContainerGroup()->childCount(); i++)
      getGroup(i)->initialize();
  }
  if(getContainerObject()) {
    for(int i=0; i<getContainerObject()->childCount(); i++)
      getObject(i)->initialize();
  }
  if(getContainerFrame()) {
    for(int i=0; i<getContainerFrame()->childCount(); i++)
      getFrame(i)->initialize();
  }
  if(getContainerLink()) {
    for(int i=0; i<getContainerLink()->childCount(); i++)
      getLink(i)->initialize();
  }
}

void Element::rename() {
  QString text = QInputDialog::getText(0, tr("Rename"), tr("Name:"), QLineEdit::Normal);

  setName(text);
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Element::initializeUsingXML(TiXmlElement *element) {
}

TiXmlElement* Element::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
  parent->LinkEndChild(ele0);
  ele0->SetAttribute("name", text(0).toStdString());
  return ele0;
}

QString Element::getXMLPath(Element *ref, bool rel) {
  if(rel) {
    vector<Element*> e0, e1;
    Element* element = ref;
    e0.push_back(element);
    while(!dynamic_cast<Solver*>(element)) {
      element = element->getParentElement();
      e0.push_back(element);
    }
    element = parentElement;
    e1.push_back(element);
    while(!dynamic_cast<Solver*>(element)) {
      element = element->getParentElement();
      e1.push_back(element);
    }
    int imatch=0;
    for(vector<Element*>::iterator i0 = e0.end()-1, i1 = e1.end()-1 ; (i0 != e0.begin()-1) && (i1 != e1.begin()-1) ; i0--, i1--) 
      if(*i0 == *i1) imatch++;
    QString type;
    if(dynamic_cast<Group*>(this))
      type = "Group";
    else if(dynamic_cast<Object*>(this))
      type = "Object";
    else 
      type = getType();
    QString str = type + "[" + getName() + "]";
    for(vector<Element*>::iterator i1 = e1.begin() ; i1 != e1.end()-imatch ; i1++) {
      if(dynamic_cast<Group*>(*i1))
        str = QString("Group[") + (*i1)->getName() + "]/" + str;
      else if(dynamic_cast<Object*>(*i1))
        str = QString("Object[") + (*i1)->getName() + "]/" + str;
      else
        throw;
    }
    for(int i=0; i<int(e0.size())-imatch; i++)
      str = "../" + str;
    return str;
  } else {
    QString type;
    if(dynamic_cast<Group*>(this))
      type = "Group";
    else if(dynamic_cast<Object*>(this))
      type = "Object";
    else 
      type = getType();
    QString str = type + "[" + getName() + "]";
    Element* element = parentElement;
    while(!dynamic_cast<Solver*>(element)) {
      if(dynamic_cast<Group*>(element))
        str = QString("Group[") + element->getName() + "]/" + str;
      else if(dynamic_cast<Object*>(element))
        str = QString("Object[") + element->getName() + "]/" + str;
      else
        throw;
      element = element->getParentElement();
    }
    str = "/" + str;
    return str;
  }
}

//std::vector<Frame*> Element::getFrames() const {
//  std::vector<Frame*> frames;
//  for(int i=0; i<childCount(); i++)
//    if(dynamic_cast<Frame*>(child(i)))
//      frames.push_back((Frame*)child(i));
//  return frames;
//}
//
//std::vector<Object*> Element::getObjects() const {
//  std::vector<Object*> objects;
//  for(int i=0; i<childCount(); i++)
//    if(dynamic_cast<Object*>(child(i)))
//      objects.push_back((Object*)child(i));
//  return objects;
//}
//
//std::vector<Group*> Element::getGroups() const {
//  std::vector<Group*> groups;
//  for(int i=0; i<childCount(); i++)
//    if(dynamic_cast<Group*>(child(i)))
//      groups.push_back((Group*)child(i));
//  return groups;
//}  
//
//std::vector<Link*> Element::getLinks() const {
//  std::vector<Link*> objects;
//  for(int i=0; i<childCount(); i++)
//    if(dynamic_cast<Link*>(child(i)))
//      objects.push_back((Link*)child(i));
//  return objects;
//}
//
//ExtraDynamic* Element::getExtraDynamic(const string &name, bool check) {
//  //    unsigned int i;
//  //    for(i=0; i<extraDynamic.size(); i++) {
//  //      if(extraDynamic[i]->getName() == name)
//  //        return extraDynamic[i];
//  //    }
//  //    if(check){
//  //      if(!(i<extraDynamic.size()))
//  //        throw MBSimError("The DynamicSystem \""+this->name+"\" comprises no ExtraDynamic \""+name+"\"!");
//  //      assert(i<extraDynamic.size());
//  //    }
//      return NULL;
//}
//
//Link* Element::getLink(const string &name, bool check) {
//    unsigned int i;
//    vector<Link*> link = getLinks();
//    for(i=0; i<link.size(); i++) {
//      if(link[i]->getName().toStdString() == name)
//        return link[i];
//    }
//    if(check){
//      if(!(i<link.size())) {
//        cout << "The DynamicSystem \""+link[i]->getName().toStdString()+"\" comprises no Link \""+name+"\"!" << endl;
//        throw;
//      }
//      assert(i<link.size());
//    }
//    return NULL;
//  }
//
//  Object* Element::getObject(const string &name, bool check) {
//    unsigned int i;
//    vector<Object*> object = getObjects();
//    for(i=0; i<object.size(); i++) {
//      if(object[i]->getName().toStdString() == name)
//        return object[i];
//    }
//    if(check){
//      if(!(i<object.size())) {
//        cout << "The DynamicSystem \""+object[i]->getName().toStdString()+"\" comprises no Object \""+name+"\"!" << endl;
//        throw;
//      }
//      assert(i<object.size());
//    }
//    return NULL;
//  }
//
//  Group* Element::getGroup(const string &name, bool check) {
//    unsigned int i;
//    vector<Group*> group = getGroups();
//    for(i=0; i<group.size(); i++) {
//      if(group[i]->getName().toStdString() == name)
//        return group[i];
//    }
//    if(check){
//      if(!(i<group.size())) {
//        cout << "The DynamicSystem \""+group[i]->getName().toStdString()+"\" comprises no DynamicSystem \""+name+"\"!";
//        throw;
//      }
//      assert(i<group.size());
//    }
//    return NULL;
//  }
//
//  Frame* Element::getFrame(const string &name, bool check) {
//    unsigned int i;
//    vector<Frame*> frame = getFrames();
//    for(i=0; i<frame.size(); i++) {
//      if(frame[i]->getName().toStdString() == name)
//        return frame[i];
//    }
//    if(check) {
//      if(!(i<frame.size())) {
//        cout << "The object \""+frame[i]->getName().toStdString()+"\" comprises no frame \""+name+"\"!" << endl;
//        throw;
//      }
//      assert(i<frame.size());
//    }
//    return NULL;
//  }

// some convenience function for XML
double Element::getDouble(TiXmlElement *e) {
  vector<vector<double> > m=strToDMat(e->GetText());
  if(m.size()==1 && m[0].size()==1)
    return m[0][0];
  else {
    ostringstream str;
    str<<": Obtained matrix of size "<<m.size()<<"x"<<m[0].size()<<" ("<<e->GetText()<<") "<<
      "where a scalar was requested for element "<<e->ValueStr();
    TiXml_location(e, "", str.str());
    throw MBSimError("Wrong type"+str.str());
  }
  return NAN;
}

int Element::getInt(TiXmlElement *e) {
  vector<vector<double> > m=strToDMat(e->GetText());
  if(m.size()==1 && m[0].size()==1)
    return lround(m[0][0]);
  else {
    ostringstream str;
    str<<": Obtained matrix of size "<<m.size()<<"x"<<m[0].size()<<" ("<<e->GetText()<<") "<<
      "where a scalar integer was requested for element "<<e->ValueStr();
    TiXml_location(e, "", str.str());
    throw MBSimError("Wrong type"+str.str());
  }
  return 0;
}

bool Element::getBool(TiXmlElement *e) {
  if(e->GetText()==string("true") || e->GetText()==string("1"))
    return true;
  else if(e->GetText()==string("false") || e->GetText()==string("0"))
    return false;
  else {
    ostringstream str;
    str<<": Obtained "<<e->GetText()<<" where a boolean was requested for element "<<e->ValueStr();
    TiXml_location(e, "", str.str());
    throw MBSimError("Wrong type"+str.str());
  }
  return 0;
}

vector<vector<double > > Element::getVec(TiXmlElement *e, int rows) {
  vector<vector<double > > m=strToDMat(e->GetText());
  if((rows==0 || m.size()==rows) && m[0].size()==1)
    return m;
  else {
    ostringstream str;
    str<<": Obtained matrix of size "<<m.size()<<"x"<<m[0].size()<<" ("<<e->GetText()<<") "<<
      "where a vector of size "<<((rows==0)?-1:rows)<<" was requested for element "<<e->ValueStr();
    TiXml_location(e, "", str.str());
    throw MBSimError("Wrong type"+str.str());
  }
  return vector<vector<double > >();
}

vector<vector<double > > Element::getMat(TiXmlElement *e, int rows, int cols) {
  vector<vector<double > > m=strToDMat(e->GetText());
  if((rows==0 || m.size()==rows) && (cols==0 || m[0].size()==cols))
    return m;
  else {
    ostringstream str;
    str<<": Obtained matrix of size "<<m.size()<<"x"<<m[0].size()<<" ("<<e->GetText()<<") "<<
      "where a matrix of size "<<((rows==0)?-1:rows)<<"x"<<((cols==0)?-1:cols)<<" was requested for element "<<e->ValueStr();
    TiXml_location(e, "", str.str());
    throw MBSimError("Wrong type"+str.str());
  }
  return vector<vector<double > >();
}

vector<vector<double > > Element::getSqrMat(TiXmlElement *e, int size) {
  vector<vector<double > > m=strToDMat(e->GetText());
  if((size==0 || m.size()==size) && (size==0 || m[0].size()==size) && m.size()==m[0].size())
    return m;
  else {
    ostringstream str;
    str<<": Obtained matrix of size "<<m.size()<<"x"<<m[0].size()<<" ("<<e->GetText()<<") "<<
      "where a square matrix of size "<<size<<" was requested for element "<<e->ValueStr();
    TiXml_location(e, "", str.str());
    throw MBSimError("Wrong type"+str.str());
  }
  return vector<vector<double > >();
}

vector<vector<double > > Element::getSymMat(TiXmlElement *e, int size) {
  vector<vector<double > > m=strToDMat(e->GetText());
  bool isSym=true;
  for(int i=0; i<min(m.size(),m[0].size()); i++) {
    for(int j=0; j<min(m.size(),m[0].size()); j++)
      if(fabs(m[i][j]-m[j][i])>1e-12) { isSym=false; break; }
    if(isSym==false) break;
  }
  if((size==0 || m.size()==size) && (size==0 || m[0].size()==size) && m.size()==m[0].size() && isSym)
    return m;
  else {
    ostringstream str;
    str<<": Obtained matrix of size "<<m.size()<<"x"<<m[0].size()<<" ("<<e->GetText()<<") "<<
      "where a symmetric matrix of size "<<size<<" was requested for element "<<e->ValueStr();
    TiXml_location(e, "", str.str());
    throw MBSimError("Wrong type"+str.str());
  }
  return vector<vector<double > >();
}

//QString Element::newName(const QString &type) {
//  bool askForName = false;
//  QString str;
//  for(int i=1; i<10000; i++) {
//    str = type + QString::number(i);
//    if(!get<Element>(str.toStdString(),false))
//      break;
//  }
//  QString text=str;
//  if(askForName) {
//    do {
//      text = QInputDialog::getText(0, tr("Add"), tr("Name:"), QLineEdit::Normal, str);
//
//      if(get<Element>(text.toStdString(),false)) {
//        QMessageBox msgBox;
//        msgBox.setText(QString("The name ") + text + " does already exist.");
//        msgBox.exec();
//      } else
//        break;
//    } while(true);
//  }
//  return text;
//}

Element* Element::getChild(QTreeWidgetItem* container, const std::string &name, bool check) {
  int i;
  for(i=0; i<container->childCount(); i++) {
    if(((Element*)container->child(i))->getName().toStdString() == name)
      return (Element*)container->child(i);
  }
  if(check) {
    if(!(i<container->childCount()))
      throw MBSimError("The object \""+((Element*)container->child(i))->getName().toStdString()+"\" comprises no frame \""+name+"\"!");
    assert(i<container->childCount());
  }
  return NULL;
}

QString Element::newName(QTreeWidgetItem* container, const QString &type) {
  bool askForName = false;
  QString str;
  for(int i=1; i<10000; i++) {
    str = type + QString::number(i);
    if(!getChild(container,str.toStdString(),false))
      break;
  }
  QString text=str;
  if(askForName) {
    do {
      text = QInputDialog::getText(0, tr("Add"), tr("Name:"), QLineEdit::Normal, str);

      if(getChild(container,text.toStdString(),false)) {
        QMessageBox msgBox;
        msgBox.setText(QString("The name ") + text + " does already exist.");
        msgBox.exec();
      } else
        break;
    } while(true);
  }
  return text;
}

Element* Container::getChild(int i) {
  return (Element*)child(i);
}

Element* Container::getChild(const std::string &name, bool check) {
  int i;
  for(i=0; i<childCount(); i++) {
    if(getChild(i)->getName().toStdString() == name)
      return (Element*)child(i);
  }
  if(check) {
    if(!(i<childCount()))
      throw MBSimError("The object \""+getChild(i)->getName().toStdString()+"\" comprises no element \""+name+"\"!");
    assert(i<childCount());
  }
  return NULL;
}

void Element::remove() {

  QTreeWidget *tree = treeWidget();
  QTreeWidgetItem::parent()->removeChild(this);
  ((Element*)tree->topLevelItem(0))->update();
}


Link* Element::getLink(const std::string &name, bool check) {
  int i;
  for(i=0; i<getContainerLink()->childCount(); i++) {
    if(getLink(i)->getName().toStdString() == name)
      return getLink(i);
  }
  if(check) {
    if(!(i<getContainerLink()->childCount()))
      throw MBSimError("The link \""+getLink(i)->getName().toStdString()+"\" comprises no frame \""+name+"\"!");
    assert(i<getContainerLink()->childCount());
  }
  return NULL;
}

Group* Element::getGroup(const std::string &name, bool check) {
  int i;
  for(i=0; i<getContainerGroup()->childCount(); i++) {
    if(getGroup(i)->getName().toStdString() == name)
      return getGroup(i);
  }
  if(check) {
    if(!(i<getContainerGroup()->childCount()))
      throw MBSimError("The group \""+getGroup(i)->getName().toStdString()+"\" comprises no frame \""+name+"\"!");
    assert(i<getContainerGroup()->childCount());
  }
  return NULL;
}

Object* Element::getObject(const std::string &name, bool check) {
  int i;
  for(i=0; i<getContainerObject()->childCount(); i++) {
    if(getObject(i)->getName().toStdString() == name)
      return getObject(i);
  }
  if(check) {
    if(!(i<getContainerObject()->childCount()))
      throw MBSimError("The object \""+getObject(i)->getName().toStdString()+"\" comprises no frame \""+name+"\"!");
    assert(i<getContainerObject()->childCount());
  }
  return NULL;
}

Frame* Element::getFrame(const std::string &name, bool check) {
  int i;
  for(i=0; i<getContainerFrame()->childCount(); i++) {
    if(getFrame(i)->getName().toStdString() == name)
      return getFrame(i);
  }
  if(check) {
    if(!(i<getContainerFrame()->childCount()))
      throw MBSimError("The frames \""+getFrame(i)->getName().toStdString()+"\" comprises no frame \""+name+"\"!");
    assert(i<getContainerFrame()->childCount());
  }
  return NULL;
}

Frame* Element::getFrame(int i) {
  return (Frame*)frames->child(i); 
}

Object* Element::getObject(int i) {
  return (Object*)objects->child(i); 
}

Link* Element::getLink(int i) {
  return (Link*)links->child(i); 
}

Group* Element::getGroup(int i) {
  return (Group*)groups->child(i); 
}


