/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#ifndef _GROUP__H_
#define _GROUP__H_

#include "element.h"
#include "group_property_dialog.h"

namespace MBSimGUI {

  class Frame;
  class Contour;
  class Object;
  class Link;
  class Observer;

  class Group : public Element {
    MBSIMGUI_OBJECTFACTORY_CLASS(Group, Element, MBSIM%"Group", "Group");
    protected:
      std::vector<Frame*> frame;
      std::vector<Contour*> contour;
      std::vector<Group*> group;
      std::vector<Object*> object;
      std::vector<Link*> link;
      std::vector<Constraint*> constraint;
      std::vector<Observer*> observer;
      xercesc::DOMElement *frames, *contours, *groups, *objects, *links, *constraints, *observers;

    public:
      Group();
      ~Group() override;
      void createXMLConstraints();
      void createXMLObservers();
      xercesc::DOMElement* getXMLFrames() override { return frames; }
      xercesc::DOMElement* getXMLContours() override { return contours; }
      xercesc::DOMElement* getXMLGroups() override { return groups; }
      xercesc::DOMElement* getXMLObjects() override { return objects; }
      xercesc::DOMElement* getXMLLinks() override { return links; }
      xercesc::DOMElement* getXMLConstraints() override { return constraints; }
      xercesc::DOMElement* getXMLObservers() override { return observers; }
      xercesc::DOMElement* getXMLFrame() override { return getXMLObservers()->getNextElementSibling(); }
      void removeXMLElements() override;
      xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent) override;
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      void create() override;
      void clear() override;
      void setDedicatedFileItem(FileItemData *dedicatedFileItem) override;
      void setDedicatedParameterFileItem(FileItemData *dedicatedFileItem) override;
      Element *getChildByContainerAndName(const QString &container, const QString &name) const override;
      void setActionPasteDisabled(bool flag);
      int getNumberOfFrames() override { return frame.size(); }
      int getNumberOfContours() override { return contour.size(); }
      int getNumberOfGroups() override { return group.size(); }
      int getNumberOfObjects() override { return object.size(); }
      int getNumberOfLinks() override { return link.size(); }
      int getNumberOfConstraints() override { return constraint.size(); }
      int getNumberOfObservers() override { return observer.size(); }
      int getIndexOfFrame(Frame *frame_) override;
      int getIndexOfContour(Contour *contour_) override;
      int getIndexOfGroup(Group *group_) override;
      int getIndexOfObject(Object *object_) override;
      int getIndexOfLink(Link *link_) override;
      int getIndexOfConstraint(Constraint *constraint_) override;
      int getIndexOfObserver(Observer *observer_) override;
      Frame* getFrame(int i) const override { return frame[i]; }
      Contour* getContour(int i) const override { return contour[i]; }
      Object* getObject(int i) const override { return object[i]; }
      Group* getGroup(int i) const override { return group[i]; }
      Link* getLink(int i) const override { return link[i]; }
      Constraint* getConstraint(int i) const override { return constraint[i]; }
      Observer* getObserver(int i) const override { return observer[i]; }
      Frame* getFrame(const QString &name) const override;
      Contour* getContour(const QString &name) const;
      Object* getObject(const QString &name) const;
      Group* getGroup(const QString &name) const;
      Link* getLink(const QString &name) const;
      Constraint* getConstraint(const QString &name) const;
      Observer* getObserver(const QString &name) const;
      void setFrame(Frame *frame_, int i) override { frame[i] = frame_; }
      void setContour(Contour *contour_, int i) override { contour[i] = contour_; }
      void setGroup(Group *group_, int i) override { group[i] = group_; }
      void setObject(Object *object_, int i) override { object[i] = object_; }
      void setLink(Link *link_, int i) override { link[i] = link_; }
      void setConstraint(Constraint *constraint_, int i) override { constraint[i] = constraint_; }
      void setObserver(Observer *observer_, int i) override { observer[i] = observer_; }
      void addFrame(Frame *frame_) override;
      void addContour(Contour *contour_) override;
      void addGroup(Group *group_) override;
      void addObject(Object *object_) override;
      void addLink(Link *link_) override;
      void addConstraint(Constraint *constraint_) override;
      void addObserver(Observer *observer_) override;
      void removeElement(Element *element) override;
      PropertyDialog* createPropertyDialog() override { return new GroupPropertyDialog(this); }
      QMenu* createFrameContextMenu() override { return new FixedRelativeFramesContextMenu(this); }
      QMenu* createContextMenu() override { return new GroupContextMenu(this); }
      void updateStatus() override;
      void createDiagramItem() override;
      void createDiagramArrows() override;
  };

  class UnknownGroup : public Group {
    MBSIMGUI_OBJECTFACTORY_CLASS(UnknownGroup, Group, MBSIM%"UnknownGroup_dummy", "Unknown group");
    public:
      UnknownGroup();
      void create() override { return Element::create(); }
      PropertyDialog* createPropertyDialog() override { return new UnknownItemPropertyDialog(this); }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement *element) override;
  };

}

#endif
