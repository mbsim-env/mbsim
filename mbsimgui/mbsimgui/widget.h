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

#ifndef _WIDGET_H_
#define _WIDGET_H_

#include <QWidget>
#include <xercesc/util/XercesDefs.hpp>
#include <mbxmlutilshelper/dom.h>
#include <objectfactory.h>
#include <namespace.h>

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
}

namespace MBSimGUI {

  class Widget : public QWidget, public ObjectFactoryBase {
    Q_OBJECT
    MBSIMGUI_OBJECTFACTORY_CLASS(Widget, ObjectFactoryBase, MBSIM%"Widget", "Widget");
    public:
      Widget(QWidget *parent=nullptr) : QWidget(parent) { }
      virtual void updateWidget() { }
      virtual void resize_(int m, int n) { }
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) { return nullptr; }
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) { return nullptr; }
    signals:
      void widgetChanged();
  };

  class WidgetFactory {
    public:
      virtual ~WidgetFactory() = default;
      virtual Widget* createWidget(int i=0) = 0;
      virtual QString getName(int i=0) const { return ""; }
      virtual int getSize() const { return 0; }
      virtual int getDefaultIndex() const { return 0; }
      virtual int getFallbackIndex() const { return getDefaultIndex(); }
      virtual int getMargin() const { return 10; }
      virtual MBXMLUtils::FQN getXMLName(int i=0) const { return ""; }
  };

  template<class Container>
  class WidgetFactoryFor : public WidgetFactory {
    public:
      WidgetFactoryFor(Element *e_=nullptr, QWidget *pw_=nullptr);
      virtual ~WidgetFactoryFor() = default;
      QString getName(int i=0) const override;
      MBXMLUtils::FQN getXMLName(int i=0) const override;
      int getDefaultIndex() const override;
      int getFallbackIndex() const override;
      Widget* createWidget(int i=0) override;
      int getSize() const override;
    private:
      Element *e;
      QWidget *pw;
  };

  template<class Container>
  WidgetFactoryFor<Container>::WidgetFactoryFor(Element *e_, QWidget *pw_) :e(e_), pw(pw_) {}

  template<class Container>
  QString WidgetFactoryFor<Container>::getName(int i) const {
    return ObjectFactory::getInstance().getAllTypesForContainer<Container>()[i]->getType();
  }

  template<class Container>
  MBXMLUtils::FQN WidgetFactoryFor<Container>::getXMLName(int i) const {
    return ObjectFactory::getInstance().getAllTypesForContainer<Container>()[i]->getXMLType();
  }

  template<class Container>
  int WidgetFactoryFor<Container>::getDefaultIndex() const {
    static int defaultIndex=-1;
    if(defaultIndex==-1) {
      defaultIndex=0;
      for(const auto *func : ObjectFactory::getInstance().getAllTypesForContainer<Container>()) {
        if(func->typeInfo==
           typeid(typename boost::mpl::at<ObjectFactory::MapContainerToDefaultAndUnknown, Container>::type::first))
          break;
        defaultIndex++;
      }
    }
    return defaultIndex;
  }

  template<class Container>
  int WidgetFactoryFor<Container>::getFallbackIndex() const {
    static int fallBackIndex=-1;
    if(fallBackIndex==-1) {
      fallBackIndex=0;
      for(const auto *func : ObjectFactory::getInstance().getAllTypesForContainer<Container>()) {
        if(func->typeInfo==
           typeid(typename boost::mpl::at<ObjectFactory::MapContainerToDefaultAndUnknown, Container>::type::second))
          break;
        fallBackIndex++;
      }
    }
    return fallBackIndex;
  }

  template<class Container>
  Widget* WidgetFactoryFor<Container>::createWidget(int i) {
    return static_cast<Container*>(ObjectFactory::getInstance().getAllTypesForContainer<Container>()[i]->ctor(e, pw));
  }

  template<class Container>
  int WidgetFactoryFor<Container>::getSize() const {
    return ObjectFactory::getInstance().getAllTypesForContainer<Container>().size();
  }

}

#endif
