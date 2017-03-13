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
#include "basic_properties.h"
#include "extended_properties.h"
#include "element_property_dialog.h"
#include "embedding_property_dialog.h"
#include "element_context_menu.h"
#include "parameter.h"

namespace MBSimGUI {

  class Element;
  class Frame;
  class Contour;
  class Group;
  class Object;
  class ExtraDynamic;
  class Link;
  class Constraint;
  class Observer;
  class TextWidget;

  namespace XERCES_CPP_NAMESPACE {
    class DOMElement;
    class DOMNode;
  }

  class Element : public TreeItemData, public PropertyInterface {
    friend class ElementPropertyDialog;
    friend class EmbeddingPropertyDialog;
    protected:
      Element *parent;
      static int IDcounter;
      std::string ID;
      ExtProperty name, embed, plotFeature;
      Parameters parameters;
      std::vector<std::string> plotFeatures;
      xercesc::DOMElement *element;
      std::string name_;
    public:
      Element(const std::string &name, Element *parent, const std::vector<MBXMLUtils::FQN> &plotFeatureTypes=std::vector<MBXMLUtils::FQN>());
      virtual ~Element() { parameters.removeParameters(); }
      virtual PropertyInterface* clone() const {return 0;}
      virtual std::string getPath();
      std::string getXMLPath(Element *ref=0, bool rel=false);
      xercesc::DOMElement* getXMLElement() { return element; }
      virtual xercesc::DOMElement* getXMLFrames() { return NULL; }
      virtual xercesc::DOMElement* getXMLObjects() { return NULL; }
      virtual xercesc::DOMElement* getXMLConstraints() { return NULL; }
      virtual xercesc::DOMElement* getXMLFrame() { return NULL; }
      virtual xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent);
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      virtual void initializeUsingXMLEmbed(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFileEmbed(xercesc::DOMNode *element);
      virtual void writeXMLFile(const std::string &name);
      virtual void writeXMLFile() { writeXMLFile(getName()); }
      virtual void writeXMLFileEmbed(const std::string &name);
      virtual void initialize() {}
      virtual void deinitialize() {}
      const std::string& getName() const { return name_; }
      void setName(const std::string &str);
      virtual std::string getType() const { return "Element"; }
      virtual std::string getValue() const;
      virtual MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIM; }
      //std::string newName(const std::string &type);
      virtual std::string getFileExtension() const { return ".xml"; }
      template<class T> T* getByPath(const std::string &path, bool initialCaller=true) const;
      virtual Element* getChildByContainerAndName(const std::string &container, const std::string &name) const { return 0; }
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
      virtual Frame* getFrame(const std::string &name) const {return 0;}
      virtual void addFrame(Frame *frame) {}
      virtual void addContour(Contour *contour) {}
      virtual void addGroup(Group *group) {}
      virtual void addObject(Object *object) {}
      virtual void addLink(Link *link) {}
      virtual void addConstraint(Constraint *constraint) {}
      virtual void addObserver(Observer *observer) {}
      virtual void removeElement(Element *element) {}
      const std::string& getID() const { return ID; }
      Element* getParent() {return parent;}
      std::vector<Element*> getParents();
      virtual void setParent(Element* parent_) {parent = parent_;}
      virtual ElementPropertyDialog* createPropertyDialog() {return new ElementPropertyDialog(this);}
      virtual EmbeddingPropertyDialog* createEmbeddingPropertyDialog() {return new EmbeddingPropertyDialog(this);}
      virtual QMenu* createContextMenu() {return new ElementContextMenu(this);}
      virtual QMenu* createEmbeddingContextMenu() {return new EmbeddingContextMenu(this);}
      virtual QMenu* createFrameContextMenu() {return NULL;}
      Element* getRoot() {return parent?parent->getRoot():this;}
      bool isEmbedded() const {return embed.isActive();}
      int getNumberOfParameters() const { return parameters.getNumberOfParameters(); }
      void addParameter(Parameter *param) { parameters.addParameter(param); embed.setActive(true); }
      void removeParameter(Parameter *param) { parameters.removeParameter(param); }
      Parameter *getParameter(int i) { return parameters.getParameter(i); }
      void setParameters(const Parameters &param) { parameters = param; }
      const Parameters& getParameters() const { return parameters; }
      std::string getCounterName() const;
      void addPlotFeature(const std::string &pf);
      const std::vector<std::string>& getPlotFeatures() const { return plotFeatures; }
  };

  template<class T>
    T* Element::getByPath(const std::string &path, bool initialCaller) const {
        if(path.substr(0, 1) == "/") { // if absolute path ...
          if(parent) // .. and a parent exists ...
            return parent->getByPath<T>(path, false); // ... than call getByPath of the parent (walk to the top)
          else // .. and no parent exits ...
            return getByPath<T>(path.substr(1), false); // ... we are at top and call getByPath again with the leading "/" removed (call relative to top)
        }
        else if (path.substr(0, 3) == "../") // if relative path to parent ...
          return parent->getByPath<T>(path.substr(3), false); // ... call getByPath of the parent with the leading "../" removed
        else { // if relative path to a child ...
          // extract the first path and all other paths (rest)
          size_t idx=path.find('/');
          std::string first=path.substr(0, idx);
          std::string rest;
          if(idx!=std::string::npos)
            rest=path.substr(idx+1);
          // get the object of the first child path by calling the virtual function getChildByContainerAndName
          size_t pos0=first.find('[');
          if(pos0==std::string::npos)
            return 0;
          std::string container=first.substr(0, pos0);
          if(first[first.size()-1]!=']')
            return 0;
          std::string name=first.substr(pos0+1, first.size()-pos0-2);
          Element *e=getChildByContainerAndName(container, name);
          // if their are other child paths call getByPath of e for this
          if(e and not(rest.empty()))
            return e->getByPath<T>(rest, false);
          // this is the last relative path -> check type and return
          return dynamic_cast<T*>(e);
        }
    }

}

#endif
