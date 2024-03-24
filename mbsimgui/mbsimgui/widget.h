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
      virtual int getStretchHint() const { return 0; }
      virtual QString getXMLComment(xercesc::DOMElement *element) { return ""; }
      virtual void setXMLComment(const QString &comment, xercesc::DOMNode *element) { }

      // get the child widget as type WidgetType
      template<class WidgetType, bool DynamicCast=false>
      WidgetType* getWidget() const {
#ifdef NDEBUG
        if constexpr(DynamicCast)
          return dynamic_cast<WidgetType*>(getWidgetVirtual());
        else
          return static_cast<WidgetType*>(getWidgetVirtual());
#else
        auto *widgetType = dynamic_cast<WidgetType*>(getWidgetVirtual());
        if(!widgetType && DynamicCast)
          return nullptr;
        assert(widgetType);
        return widgetType;
#endif
      }

      // get the child widget i as type WidgetType
      template<class WidgetType, bool DynamicCast=false>
      WidgetType* getWidget(int i) const {
#ifdef NDEBUG
        if constexpr (DynamicCast)
          return dynamic_cast<WidgetType*>(getWidgetVirtual(i));
        else
          return static_cast<WidgetType*>(getWidgetVirtual(i));
#else
        auto *widgetType = dynamic_cast<WidgetType*>(getWidgetVirtual(i));
        if(!widgetType && DynamicCast)
          return nullptr;
        assert(widgetType);
        return widgetType;
#endif
      }

      // gets the first child widget which is of type WidgetType (asserts in debug builds)
      // (this is only possible if all child widgets up to the searched widget hold only a single child widget; getWidget() is used!)
      template<class WidgetType, bool AllowNullptr=false>
      WidgetType* getFirstWidget() const {
        auto *widget = getWidgetVirtual();
        if constexpr (AllowNullptr)
          return nullptr;
        else
          throw std::runtime_error("No child widget of specified type found!");
        if(auto *widgetType = dynamic_cast<WidgetType*>(widget); widgetType)
          return widgetType;
        return widget->getFirstWidget<WidgetType>();
      }

    private:
      virtual Widget* getWidgetVirtual() const { return nullptr; }
      virtual Widget* getWidgetVirtual(int i) const { return nullptr; }

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
      virtual int getMargin() const;
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

}

#endif
