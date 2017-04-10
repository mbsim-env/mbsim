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
#include "element_property_dialog.h"
#include "embedding_property_dialog.h"
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
  class Parameter;

  namespace XERCES_CPP_NAMESPACE {
    class DOMElement;
    class DOMNode;
  }

  class Element : public TreeItemData {
    protected:
      Element *parent;
      static int IDcounter;
      std::vector<Parameter*> parameter;
      std::vector<Parameter*> removedParameter;
      std::vector<QString> plotFeatures;
      xercesc::DOMElement *element;
      QString ID, name, value, counterName;
      bool config;
    public:
      Element(const QString &name="");
      virtual ~Element();
      QString getXMLPath(Element *ref=0, bool rel=false);
      xercesc::DOMElement* getXMLElement() { return element; }
      virtual xercesc::DOMElement* getXMLFrames() { return NULL; }
      virtual xercesc::DOMElement* getXMLContours() { return NULL; }
      virtual xercesc::DOMElement* getXMLGroups() { return NULL; }
      virtual xercesc::DOMElement* getXMLObjects() { return NULL; }
      virtual xercesc::DOMElement* getXMLLinks() { return NULL; }
      virtual xercesc::DOMElement* getXMLConstraints() { return NULL; }
      virtual xercesc::DOMElement* getXMLObservers() { return NULL; }
      virtual xercesc::DOMElement* getXMLFrame() { return NULL; }
      virtual xercesc::DOMElement* processFileID(xercesc::DOMElement* element) { return element; }
      virtual xercesc::DOMElement* processHref(xercesc::DOMElement* element);
      virtual void removeXMLElements();
      virtual xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent);
      void removeXMLElement();
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      const QString& getName() const { return name; }
      void setName(const QString &str) { name = str; }
      const QString& getValue() const { return value; }
      void setValue(const QString &str) { value = str; }
      const QString& getCounterName() const { return counterName; }
      void setCounterName(const QString &str) { counterName = str; }
      QString getHref() const;
      void setHref(const QString &str);
      QString getParameterHref() const;
      void setParameterHref(const QString &str);
      virtual MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIM; }
      virtual QString getFileExtension() const { return ".xml"; }
      template<class T> T* getByPath(const QString &path, bool initialCaller=true) const;
      virtual Element* getChildByContainerAndName(const QString &container, const QString &name) const { return 0; }
      virtual int getNumberOfFrames() {return 0;}
      virtual int getNumberOfContours() {return 0;}
      virtual int getNumberOfGroups() {return 0;}
      virtual int getNumberOfObjects() {return 0;}
      virtual int getNumberOfExtraDynamics() {return 0;}
      virtual int getNumberOfLinks() {return 0;}
      virtual int getNumberOfConstraints() {return 0;}
      virtual int getNumberOfObservers() {return 0;}
      virtual Frame* getFrame(int i) const {return 0;}
      virtual Contour* getContour(int i) const {return 0;}
      virtual Group* getGroup(int i) const {return 0;}
      virtual Object* getObject(int i) const {return 0;}
      virtual Link* getLink(int i) const {return 0;}
      virtual Constraint* getConstraint(int i) const {return 0;}
      virtual Observer* getObserver(int i) const {return 0;}
      virtual Frame* getFrame(const QString &name) const {return 0;}
      virtual void addFrame(Frame *frame) {}
      virtual void addContour(Contour *contour) {}
      virtual void addGroup(Group *group) {}
      virtual void addObject(Object *object) {}
      virtual void addLink(Link *link) {}
      virtual void addConstraint(Constraint *constraint) {}
      virtual void addObserver(Observer *observer) {}
      virtual void removeElement(Element *element) {}
      const QString& getID() const { return ID; }
      void setID(const QString &ID_) { ID = ID_; }
      Element* getParent() {return parent;}
      std::vector<Element*> getParents();
      void setParent(Element* parent_) {parent = parent_;}
      virtual ElementPropertyDialog* createPropertyDialog() {return new ElementPropertyDialog(this);}
      virtual EmbeddingPropertyDialog* createEmbeddingPropertyDialog() {return new EmbeddingPropertyDialog(this);}
      virtual QMenu* createContextMenu() {return new ElementContextMenu(this);}
      virtual QMenu* createEmbeddingContextMenu() {return new EmbeddingContextMenu(this);}
      virtual QMenu* createFrameContextMenu() {return NULL;}
      Element* getRoot() {return parent?parent->getRoot():this;}
      void addParameter(Parameter *param);
      void removeParameter(Parameter *param);
      int getNumberOfParameters() const { return parameter.size(); }
      Parameter *getParameter(int i) { return parameter[i]; }
      void addPlotFeature(const QString &pf);
      const std::vector<QString>& getPlotFeatures() const { return plotFeatures; }
      virtual QString getPlotFeatureType() const { return ""; }
      bool getConfig() { return config; }
      void setConfig(bool config_) { config = config_; }
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
            return 0;
          QString container=first.mid(0, pos0);
          if(first[first.size()-1]!=']')
            return 0;
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
