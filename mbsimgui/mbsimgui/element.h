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

#ifndef _ELEMENT__H_
#define _ELEMENT__H_

#include "embeditemdata.h"
#include "element_property_dialog.h"
#include "element_context_menu.h"
#include "namespace.h"

namespace MBSimGUI {

  class Frame;
  class Contour;
  class Group;
  class Object;
  class Link;
  class Constraint;
  class Observer;

  class Element : public EmbedItemData {
    MBSIMGUI_OBJECTFACTORY_CLASS(Element, EmbedItemData, MBSIM%"Element", "Element");
    protected:
      Element *parent{nullptr};
      std::vector<MBXMLUtils::FQN> plotFeatures;
      std::string ID;
      bool enabled{true};
      void emitDataChangedOnChildren();
    public:
      Element();
      virtual QString getXMLName();
      QString getXMLPath(Element *ref=nullptr, bool rel=false);
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      virtual xercesc::DOMElement* getXMLFrames() { return nullptr; }
      virtual xercesc::DOMElement* getXMLContours() { return nullptr; }
      virtual xercesc::DOMElement* getXMLGroups() { return nullptr; }
      virtual xercesc::DOMElement* getXMLObjects() { return nullptr; }
      virtual xercesc::DOMElement* getXMLLinks() { return nullptr; }
      virtual xercesc::DOMElement* getXMLConstraints() { return nullptr; }
      virtual xercesc::DOMElement* getXMLObservers() { return nullptr; }
      virtual xercesc::DOMElement* getXMLFrame() { return nullptr; }
      virtual xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent);
      template<class T> T* getByPath(const QString &path, bool initialCaller=true) const;
      virtual Element* getChildByContainerAndName(const QString &container, const QString &name) const { return nullptr; }
      virtual int getNumberOfFrames() { return 0; }
      virtual int getNumberOfContours() { return 0; }
      virtual int getNumberOfGroups() { return 0; }
      virtual int getNumberOfObjects() { return 0; }
      virtual int getNumberOfLinks() { return 0; }
      virtual int getNumberOfConstraints() { return 0; }
      virtual int getNumberOfObservers() { return 0; }
      virtual int getIndexOfFrame(Frame *frame) { return -1; }
      virtual int getIndexOfContour(Contour *contour) { return -1; }
      virtual int getIndexOfGroup(Group *group) { return -1; }
      virtual int getIndexOfObject(Object *object) { return -1; }
      virtual int getIndexOfLink(Link *link) { return -1; }
      virtual int getIndexOfConstraint(Constraint *constraint) { return -1; }
      virtual int getIndexOfObserver(Observer *observer) { return -1; }
      virtual Frame* getFrame(int i) const { return nullptr; }
      virtual Contour* getContour(int i) const { return nullptr; }
      virtual Group* getGroup(int i) const { return nullptr; }
      virtual Object* getObject(int i) const { return nullptr; }
      virtual Link* getLink(int i) const { return nullptr; }
      virtual Constraint* getConstraint(int i) const { return nullptr; }
      virtual Observer* getObserver(int i) const { return nullptr; }
      virtual Frame* getFrame(const QString &name) const { return nullptr; }
      virtual void setFrame(Frame *frame, int i) { }
      virtual void setContour(Contour *contour, int i) { }
      virtual void setGroup(Group *group, int i) { }
      virtual void setObject(Object *object, int i) { }
      virtual void setLink(Link *link, int i) { }
      virtual void setConstraint(Constraint *constraint, int i) { }
      virtual void setObserver(Observer *observer, int i) { }
      virtual void addFrame(Frame *frame) { }
      virtual void addContour(Contour *contour) { }
      virtual void addGroup(Group *group) { }
      virtual void addObject(Object *object) { }
      virtual void addLink(Link *link) { }
      virtual void addConstraint(Constraint *constraint) { }
      virtual void addObserver(Observer *observer) { }
      virtual void removeElement(Element *element) { }
      const std::string& getID() const { return ID; }
      void setID(const std::string &ID_) { ID = ID_; }
      Element* getParent() { return parent; }
      EmbedItemData* getEmbedItemParent() override { return getParent(); }
      void setParent(Element* parent_) { parent = parent_; }
      PropertyDialog* createPropertyDialog() override { return new ElementPropertyDialog(this); }
      QMenu* createContextMenu() override { return new ElementContextMenu(this); }
      virtual QMenu* createFrameContextMenu() { return nullptr; }
      const std::vector<MBXMLUtils::FQN>& getPlotFeatures() const { return plotFeatures; }
      virtual MBXMLUtils::FQN getPlotFeatureType() const { return ""; }
      bool getEnabled() const override { return enabled; }
      void updateStatus() override;
      EmbedItemData *getDedicatedItem() override { return dedicatedFileItem?(fileItem?this:parent->getDedicatedItem()):this; }
  };

  template<class T>
    T* Element::getByPath(const QString &path, bool initialCaller) const {
        if(path.mid(0, 1) == "/") { // if absolute path ...
          if(parent) // .. and a parent exists ...
            return parent->getByPath<T>(path, false); // ... than call getByPath of the parent (walk to the top)
          else // .. and no parent exits ...
            return getByPath<T>(path.mid(1), false); // ... we are at top and call getByPath again with the leading "/" removed (call relative to top)
        }
        else if (path.mid(0, 3) == "../") // if relative path to parent ...
          return parent->getByPath<T>(path.mid(3), false); // ... call getByPath of the parent with the leading "../" removed
        else { // if relative path to a child ...
          // extract the first path and all other paths (rest)
          size_t idx=path.indexOf('/');
          QString first=path.mid(0, idx);
          QString rest;
          if(idx!=-1)
            rest=path.mid(idx+1);
          // get the object of the first child path by calling the virtual function getChildByContainerAndName
          size_t pos0=first.indexOf('[');
          if(pos0==-1)
            return nullptr;
          QString container=first.mid(0, pos0);
          if(first[first.size()-1]!=']')
            return nullptr;
          QString name=first.mid(pos0+1, first.size()-pos0-2);
          Element *e=getChildByContainerAndName(container, name);
          // if their are other child paths call getByPath of e for this
          if(e and not(rest.isEmpty()))
            return e->getByPath<T>(rest, false);
          // this is the last relative path -> check type and return
          return dynamic_cast<T*>(e);
        }
    }

}

#endif
