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

#ifndef _GROUP__H_
#define _GROUP__H_

#include "element.h"
#include "extended_properties.h"
class Frame;
class Contour;
class Object;
class Link;
class Observer;

class Group : public Element {
  friend class GroupPropertyDialog;
  protected:
    ExtProperty position, orientation, frameOfReference; 
    std::vector<Frame*> frame;
    std::vector<Contour*> contour;
    std::vector<Group*> group;
    std::vector<Object*> object;
    std::vector<ExtraDynamic*> extraDynamic;
    std::vector<Link*> link;
    std::vector<Observer*> observer;
    std::vector<Element*> removedElement;

  public:
    Group(const std::string &str, Element *parent);
    ~Group();
    std::string getType() const { return "Group"; }
    int getqSize();
    int getuSize();
    int getxSize();
    static Group* readXMLFile(const std::string &filename, Element *parent);
    virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    virtual Element *getByPathSearch(std::string path);
    void setActionPasteDisabled(bool flag);
    void initialize();
    int getNumberOfFrames() {return frame.size();}
    int getNumberOfContours() {return contour.size();}
    int getNumberOfGroups() {return group.size();}
    int getNumberOfObjects() {return object.size();}
    int getNumberOfExtraDynamics() {return extraDynamic.size();}
    int getNumberOfLinks() {return link.size();}
    int getNumberOfObservers() {return observer.size();}
    Frame* getFrame(int i) {return frame[i];}
    Contour* getContour(int i) {return contour[i];}
    Object* getObject(int i) {return object[i];}
    Group* getGroup(int i) {return group[i];}
    ExtraDynamic* getExtraDynamic(int i) {return extraDynamic[i];}
    Link* getLink(int i) {return link[i];}
    Observer* getObserver(int i) {return observer[i];}
    Frame* getFrame(const std::string &name);
    Contour* getContour(const std::string &name);
    Object* getObject(const std::string &name);
    Group* getGroup(const std::string &name);
    ExtraDynamic* getExtraDynamic(const std::string &name);
    Link* getLink(const std::string &name);
    Observer* getObserver(const std::string &name);

    void addFrame(Frame *frame);
    void addContour(Contour *contour);
    void addGroup(Group *group);
    void addObject(Object *object);
    void addExtraDynamic(ExtraDynamic *extraDynamic);
    void addLink(Link *link);
    void addObserver(Observer *observer);
    void removeElement(Element *element);
    ElementPropertyDialog* createPropertyDialog() {return new GroupPropertyDialog(this);}
    ElementContextMenu* createContextMenu() {return new GroupContextMenu(this);}
};

#endif
