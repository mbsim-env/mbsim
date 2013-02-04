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
#include "property_widget.h"
#include "utils.h"

class Element;
class Frame;
class Contour;
class Group;
class Object;
class Link;
class ExtraDynamic;
class TiXmlElement;
class TiXmlNode;

class Container : public QTreeWidgetItem {
  public:
    Element* getChild(int i);
    Element* getChild(const QString &name, bool check=false);
};

class Element : public QObject, public QTreeWidgetItem {
  Q_OBJECT
  protected:
    bool drawThisPath;
    QString iconFile;
    bool searchMatched;
    PropertyWidget *properties;
    QMenu *contextMenu;
    QAction *actionSave;
    QString file;
    ExtXMLWidget *name;
    std::vector<ExtXMLWidget*> plotFeature;
    static TiXmlElement* copiedElement;
    Element *parentElement;
    Container *frames, *contours, *groups, *objects, *links, *extraDynamics;
    static int IDcounter;
    std::string ns, ID;
  public:
    Element(const QString &str, QTreeWidgetItem *parentItem, int ind, bool grey=false);
    virtual ~Element();
    Element* getParentElement() {return parentElement;}
    virtual QString getPath();
    QString getXMLPath(Element *ref=0, bool rel=false);
    QString &getIconFile() { return iconFile; }
    virtual QString getInfo();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual void writeXMLFile(const QString &name);
    virtual void writeXMLFile() { writeXMLFile(getName()); }
    virtual void update();
    virtual void initialize();
    virtual void resizeVariables();
    virtual QString getType() const { return "Element"; }
    //QString newName(const QString &type);
    virtual QString getFileExtension() const { return ".xml"; }
    void updateTextColor();
    bool getSearchMatched() { return searchMatched; }
    void setSearchMatched(bool m) { searchMatched=m; }
    QMenu* getContextMenu() { return contextMenu; }
    PropertyWidget* getPropertyWidget() { return properties; }
    QString getName() const {return text(0);}
    void setName(const QString &str);
    template<class T> T* getByPath(QString path);
    virtual Element* getByPathSearch(QString path) {return 0; }
    Element* getChild(QTreeWidgetItem* container, const QString &name, bool check=true);
    QString newName(QTreeWidgetItem* container, const QString &type);
    virtual Container* getContainerFrame() {return frames;}
    virtual Container* getContainerContour() {return contours;}
    virtual Container* getContainerGroup() {return groups;}
    virtual Container* getContainerObject() {return objects;}
    virtual Container* getContainerLink() {return links;}
    virtual Frame* getFrame(int i);
    virtual Contour* getContour(int i);
    virtual Group* getGroup(int i);
    virtual Object* getObject(int i);
    virtual Link* getLink(int i);
    Frame* getFrame(const QString &name, bool check=true);
    Contour* getContour(const QString &name, bool check=true);
    Object* getObject(const QString &name, bool check=true);
    Group* getGroup(const QString &name, bool check=true);
    Link* getLink(const QString &name, bool check=true);
    std::string getID() { return ID; }
    static std::map<std::string, Element*> idEleMap;
  public slots:
    void remove();
    virtual void saveAs();
    virtual void save();
    void copy();
};

template<class T>
T* Element::getByPath(QString path) {
  Element * e = getByPathSearch(path);
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
//T* Element::get(const QString &name, bool check) {
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
