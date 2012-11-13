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

#ifndef _ELEMENT__H_
#define _ELEMENT__H_

#include <QtGui/QTreeWidgetItem>
#include "mbsimguitinyxml/tinyxml-src/tinyxml.h"
#include "mbsimguitinyxml/tinyxml-src/tinynamespace.h"
#include "editors.h"
#include "utils.h"
#include <string>
#include <set>

#define MBSIMNS_ "http://mbsim.berlios.de/MBSim"
#define MBSIMNS "{"MBSIMNS_"}"

class PropertyDialog;
class Frame;
class Group;
class Object;
class Link;
class ExtraDynamic;

class Container : public QTreeWidgetItem {
  public:
    Element* getChild(int i);
    Element* getChild(const std::string &name, bool check=false);
};


class Element : public QObject, public QTreeWidgetItem {
  Q_OBJECT
  friend class Editor;
  friend class MainWindow;
  protected:
    bool drawThisPath;
    std::string iconFile;
    bool searchMatched;
    PropertyDialog *properties;
    QMenu *contextMenu;
    QAction *actionSave;
    QString file;
    XMLEditor *name;
    static TiXmlElement* copiedElement;
    Element *parentElement;
    Container *frames, *contours, *groups, *objects, *links, *extraDynamics;
  public:
    Element(const QString &str, QTreeWidgetItem *parentItem, int ind, bool grey=false);
    virtual ~Element();
    Element* getParentElement() {return parentElement;}
    virtual QString getPath();
    QString getXMLPath(Element *ref=0, bool rel=false);
    std::string &getIconFile() { return iconFile; }
    virtual QString getInfo();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual void writeXMLFile(const QString &name);
    virtual void writeXMLFile() { writeXMLFile(getName()); }
    virtual void update();
    virtual QString getType() const { return "Element"; }
    //QString newName(const QString &type);
    virtual QString getFileExtension() const { return ".xml"; }
    void updateTextColor();
    bool getSearchMatched() { return searchMatched; }
    void setSearchMatched(bool m) { searchMatched=m; }
    QMenu* getContextMenu() { return contextMenu; }
    PropertyDialog* getPropertyDialog() { return properties; }
    QString getName() const {return text(0);}
    void setName(const QString &str) {setText(0,str);((NameWidget*)name->getXMLWidget())->setName(str);}
    static double getDouble(TiXmlElement *e);
    static int getInt(TiXmlElement *e);
    static bool getBool(TiXmlElement *e);
    static std::vector<std::vector<double > > getVec(TiXmlElement *e, int rows=0);
    static std::vector<std::vector<double > > getMat(TiXmlElement *e, int rows=0, int cols=0);
    static std::vector<std::vector<double > > getSqrMat(TiXmlElement *e, int size=0);
    static std::vector<std::vector<double > > getSymMat(TiXmlElement *e, int size=0);
    template<class T> T* getByPath(QString path);
    virtual Element* getByPathSearch(std::string path) {return 0; }
    virtual void initialize();
    Element* getChild(QTreeWidgetItem* container, const std::string &name, bool check=true);
    QString newName(QTreeWidgetItem* container, const QString &type);
    virtual Container* getContainerFrame() {return frames;}
    virtual Container* getContainerGroup() {return groups;}
    virtual Container* getContainerObject() {return objects;}
    virtual Container* getContainerLink() {return links;}
    virtual Frame* getFrame(int i);
    virtual Group* getGroup(int i);
    virtual Object* getObject(int i);
    virtual Link* getLink(int i);
    Frame* getFrame(const std::string &name, bool check=true);
    Object* getObject(const std::string &name, bool check=true);
    Group* getGroup(const std::string &name, bool check=true);
    Link* getLink(const std::string &name, bool check=true);
    public slots:
      void remove();
      virtual void saveAs();
      virtual void save();
      void copy();
};

template<class T>
T* Element::getByPath(QString path) {
  Element * e = getByPathSearch(path.toStdString());
  if (dynamic_cast<T*>(e))
    return (T*)(e);
  else
    throw MBSimError("ERROR in "+getName().toStdString()+" (Element::getByPath): Element \""+path.toStdString()+"\" not found or not of wanted type!");
}

//template<class T>
//std::vector<T*> Element::get() const {
//  std::vector<T*> elements;
//  for(int i=0; i<childCount(); i++)
//    if(dynamic_cast<T*>(child(i)))
//      elements.push_back((T*)child(i));
//  return elements;
//}
//
//template<class T>
//T* Element::get(const std::string &name, bool check) {
//  unsigned int i;
//  std::vector<T*> element = get<T>();
//  for(i=0; i<element.size(); i++) {
//    if(element[i]->getName().toStdString() == name)
//      return element[i];
//  }
//  if(check) {
//    if(!(i<element.size()))
//      throw MBSimError("The object \""+element[i]->getName().toStdString()+"\" comprises no element \""+name+"\"!");
//    assert(i<element.size());
//  }
//  return NULL;
//}


#endif
