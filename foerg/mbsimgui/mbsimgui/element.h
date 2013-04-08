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

#include "treeitemdata.h"
#include "utils.h"
#include "extended_properties.h"
#include "property_widget.h"

class Element;
class Frame;
class Contour;
class Group;
class Object;
class Link;
class ExtraDynamic;
class Observer;
class TiXmlElement;
class TiXmlNode;
class TextWidget;

class Element : public TreeItemData {
  protected:
    bool drawThisPath;
    std::string iconFile;
    bool searchMatched;
    std::string file;
    static TiXmlElement* copiedElement;
    static int IDcounter;
    std::string ns, ID;
    std::string name;
    Element *parent;
  public:
    Element(const std::string &name, Element *parent);
    virtual ~Element();
    virtual std::string getPath();
    std::string getXMLPath(Element *ref=0, bool rel=false);
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual void writeXMLFile(const std::string &name);
    virtual void writeXMLFile() { writeXMLFile(getName()); }
    virtual void initialize();
    const std::string& getName() const {return name;}
    void setName(const std::string &str) {name = str;}
    const std::string getType() const { return "Element"; }
    //std::string newName(const std::string &type);
    virtual std::string getFileExtension() const { return ".xml"; }
    bool getSearchMatched() { return searchMatched; }
    void setSearchMatched(bool m) { searchMatched=m; }
    template<class T> T* getByPath(std::string path);
    virtual Element* getByPathSearch(std::string path) {return 0; }
    //Element* getChild(TreeItem* container, const std::string &name, bool check=true);
    //virtual Container* getContainerFrame() {return frames;}
    //virtual Container* getContainerContour() {return contours;}
    //virtual Container* getContainerGroup() {return groups;}
    //virtual Container* getContainerObject() {return objects;}
    //virtual Container* getContainerLink() {return links;}
    //virtual Container* getContainerObserver() {return observers;}
    virtual int getNumberOfFrames() {return 0;}
    virtual Frame* getFrame(int i) {return 0;}
    //virtual Contour* getContour(int i);
    //virtual Group* getGroup(int i);
    //virtual Object* getObject(int i);
    //virtual Link* getLink(int i);
    //virtual Observer* getObserver(int i);
    virtual Frame* getFrame(const std::string &name, bool check=true) {return 0;}
    //Contour* getContour(const std::string &name, bool check=true);
    //Object* getObject(const std::string &name, bool check=true);
    //Group* getGroup(const std::string &name, bool check=true);
    //Link* getLink(const std::string &name, bool check=true);
    //Observer* getObserver(const std::string &name, bool check=true);
    virtual void addFrame(Frame *frame) {}
    std::string getID() { return ID; }
    static std::map<std::string, Element*> idEleMap;
    virtual Element* getParent() {return parent;}
    virtual void setParent(Element* parent_) {parent = parent_;}
    PropertyDialog* createPropertyDialog() {return new ElementPropertyDialog;}
};

template<class T>
T* Element::getByPath(std::string path) {
  Element * e = getByPathSearch(path);
  if (dynamic_cast<T*>(e))
    return (T*)(e);
  else
    throw MBSimError("ERROR in "+getName()+" (Element::getByPath): Element \""+path+"\" not found or not of wanted type!");
}

#endif
