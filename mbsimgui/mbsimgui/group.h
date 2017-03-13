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

namespace MBSimGUI {

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
    std::vector<Link*> link;
    std::vector<Constraint*> constraint;
    std::vector<Observer*> observer;
    std::vector<Element*> removedElement;

    public:
    Group(const std::string &str, Element *parent);
    Group(const Group &g);
    ~Group();
    Group& operator=(const Group &g);
    virtual PropertyInterface* clone() const {return new Group(*this);}
    std::string getType() const { return "Group"; }
    int getqSize();
    int getuSize();
    int getxSize();
    static Group* readXMLFile(const std::string &filename, Element *parent);
    virtual xercesc::DOMElement* getXMLObjects();
      virtual xercesc::DOMElement* getXMLConstraints();
    virtual xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent);
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    virtual Element *getChildByContainerAndName(const std::string &container, const std::string &name) const;
    void setActionPasteDisabled(bool flag);
    void initialize();
    int getNumberOfFrames() {return frame.size();}
    int getNumberOfContours() {return contour.size();}
    int getNumberOfGroups() {return group.size();}
    int getNumberOfObjects() {return object.size();}
    int getNumberOfLinks() {return link.size();}
    int getNumberOfConstraints() {return constraint.size();}
    int getNumberOfObservers() {return observer.size();}
    Frame* getFrame(int i) const {return frame[i];}
    Contour* getContour(int i) const {return contour[i];}
    Object* getObject(int i) const {return object[i];}
    Group* getGroup(int i) const {return group[i];}
    Link* getLink(int i) const {return link[i];}
    Constraint* getConstraint(int i) const {return constraint[i];}
    Observer* getObserver(int i) const {return observer[i];}
    Frame* getFrame(const std::string &name) const;
    Contour* getContour(const std::string &name) const;
    Object* getObject(const std::string &name) const;
    Group* getGroup(const std::string &name) const;
    Link* getLink(const std::string &name) const;
    Constraint* getConstraint(const std::string &name) const;
    Observer* getObserver(const std::string &name) const;

    void addFrame(Frame *frame);
    void addContour(Contour *contour);
    void addGroup(Group *group);
    void addObject(Object *object);
    void addLink(Link *link);
    void addConstraint(Constraint *constraint);
    void addObserver(Observer *observer);
    void removeElement(Element *element);
    ElementPropertyDialog* createPropertyDialog() {return new GroupPropertyDialog(this);}
    QMenu* createContextMenu() {return new GroupContextMenu(this);}
    QMenu* createFrameContextMenu() {return new FixedRelativeFrameContextContextMenu(this);}
  };

}

#endif
