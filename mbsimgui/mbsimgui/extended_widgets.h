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

#ifndef _EXTENDED_WIDGETS_H_
#define _EXTENDED_WIDGETS_H_

#include "widget.h"
#include <optional>
#include <QLabel>
#include <QComboBox>
#include <QBoxLayout>

class QSpinBox;
class QStackedWidget;
class QListWidget;
class QPushButton;

namespace MBSimGUI {

  class MouseEvent : public QObject {
    Q_OBJECT
    public:
      MouseEvent(QWidget* widget) : QObject(widget) { }
    protected:
      bool eventFilter(QObject *obj, QEvent *event) override;
    signals:
      void mouseButtonPressed();
  };

  class ExtWidget : public Widget {
    Q_OBJECT

    public:
      ExtWidget(const QString &name, Widget *widget_, bool checkable_=false, bool active=false, MBXMLUtils::FQN xmlName_="", bool comment=false, const QString &defaultEmployed="(default employed)");
      int getStretchHint() const override { return widget->getStretchHint(); }
      void resize_(int m, int n) override { if(isActive()) widget->resize_(m,n); }
      void setActive(bool active);
      bool isActive() const { return not checkable or checked; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      void updateWidget() override { if(isActive()) widget->updateWidget(); }

    protected:
      bool checkable, checked;
      Widget *widget;
      MBXMLUtils::FQN xmlName;
      static std::optional<QPixmap> expandedPixmap, collapsedPixmap; // this optional is initialed in the first ctor call (cannot be done static since Qt must be init first)
      static std::optional<QIcon> commentIcon, noCommentIcon; // this optional is initialed in the first ctor call (cannot be done static since Qt must be init first)
      QLabel *iconLabel; // the icon of a optional Widget (first column)
      // QLabel textLabel; // the name of the optional Widget (second column)
      QLabel *defaultLabel; // the "(default employed)" text of the optional Widget (third column)
      QPushButton *commentButton{nullptr};
      QString comment;
      void setComment(const QString &comment);
      void editComment();

    signals:
      void clicked(bool);

    private:
      Widget* getWidgetVirtual() const override { return widget; }
  };

  class ChoiceWidget : public Widget {
    Q_OBJECT

    public:
      ChoiceWidget(WidgetFactory *factory_, QBoxLayout::Direction dir=QBoxLayout::TopToBottom, int mode_=4);
      int getStretchHint() const override { return widget ? widget->getStretchHint() : 0; }
      void updateWidget() override { widget->updateWidget(); }
      QString getName() const { return comboBox->currentText(); }
      int getIndex() const { return comboBox->currentIndex(); }
      void setIndex(int i) { return comboBox->setCurrentIndex(i); }
      void resize_(int m, int n) override { widget->resize_(m,n); }
      void setWidgetFactory(WidgetFactory *factory_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      void defineWidget(int);
      QString getXMLComment(xercesc::DOMElement *element) override;
      void setXMLComment(const QString &comment, xercesc::DOMNode *element) override;

      template<class WidgetFactoryType, bool DynamicCast=false>
      WidgetFactoryType* getWidgetFactory() const {
#ifdef NDEBUG
        if constexpr(DynamicCast)
          return dynamic_cast<WidgetFactoryType*>(factory);
        else
          return static_cast<WidgetFactoryType*>(factory);
#else
        auto *widgetFactoryType = dynamic_cast<WidgetFactoryType*>(factory);
        if(!widgetFactoryType && DynamicCast)
          return nullptr;
        assert(widgetFactoryType);
        return widgetFactoryType;
#endif
      }

    protected:
      QBoxLayout *layout;
      QComboBox *comboBox;
      Widget *widget;
      WidgetFactory *factory;
      int mode;

    signals:
      void comboChanged(int);

    private:
      Widget* getWidgetVirtual() const override { return widget; }
  };

  class ContainerWidget : public Widget {

    public:
      ContainerWidget();

      void resize_(int m, int n) override;
      void addWidget(Widget *widget_);
      void updateWidget() override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QBoxLayout *layout;
      std::vector<Widget*> widget;

    private:
      Widget* getWidgetVirtual(int i) const override { return widget[i]; }
  };

  class ListWidget : public Widget {

    public:
      ListWidget(WidgetFactory *factory_, const QString &name_="Element", int m=0, int mode_=0, bool fixedSize=false, int minSize=0, int maxSize=100);
      ~ListWidget() override;
      void resize_(int m, int n) override;
      int getSize() const;
      void setSize(int m);
      void setRange(int minSize, int maxSize);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      QString getXMLComment(xercesc::DOMElement *element) override;
      void setXMLComment(const QString &comment, xercesc::DOMNode *element) override;

    protected:
      void addElements(int n=1, bool emitSignals=true);
      void removeElements(int n=1);
      void changeCurrent(int idx);
      void changeSize(int size);
      QStackedWidget *stackedWidget;
      QSpinBox* spinBox;
      QListWidget *list;
      WidgetFactory *factory;
      QString name;
      int mode;

    private:
      Widget* getWidgetVirtual(int i) const override;
  };

  class ChoiceWidgetFactory : public WidgetFactory {
    public:
      ChoiceWidgetFactory(WidgetFactory *factory_, QBoxLayout::Direction dir_=QBoxLayout::TopToBottom, int mode_=1) : factory(factory_), dir(dir_), mode(mode_) { }
      Widget* createWidget(int i=0) override;
      MBXMLUtils::FQN getXMLName(int i=0) const override { return factory->getXMLName(i); }
    protected:
      WidgetFactory *factory;
      QBoxLayout::Direction dir;
      int mode;
  };

}

#endif
