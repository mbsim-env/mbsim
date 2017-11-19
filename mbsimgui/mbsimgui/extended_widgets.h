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

#ifndef _EXTENDED_WIDGETS_H_
#define _EXTENDED_WIDGETS_H_

#include "widget.h"
#include <QComboBox>
#include <QGroupBox>
#include <QVBoxLayout>

class QSpinBox;
class QStackedWidget;
class QListWidget;

namespace MBSimGUI {

  class VariableWidget;
  class PhysicalVariableWidget;
  class EvalDialog;

  class ExtWidget : public QGroupBox, public WidgetInterface {
    Q_OBJECT

    public:
      ExtWidget(const QString &name, QWidget *widget, bool deactivatable=false, bool active=false, MBXMLUtils::FQN xmlName="");
      QWidget* getWidget() const {return widget;}
      void resize_(int m, int n) override {if(isActive()) dynamic_cast<WidgetInterface*>(widget)->resize_(m,n);}
      bool isActive() const {return (isCheckable() && !isChecked())?false:true;}
      void setActive(bool flag) {if(isCheckable()) setChecked(flag);}
      void setWidgetVisible(bool flag) {if(isCheckable()) widget->setVisible(flag);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QWidget *widget;
      MBXMLUtils::FQN xmlName;
    public slots:
      void updateWidget() override {if(isActive()) dynamic_cast<WidgetInterface*>(widget)->updateWidget();}
    signals:
      void widgetChanged();
  };

  class ChoiceWidget2 : public Widget {
    Q_OBJECT

    public:
      ChoiceWidget2(WidgetFactory *factory, QBoxLayout::Direction dir=QBoxLayout::TopToBottom, int mode=4);
      QWidget* getWidget() const { return widget; }
      void updateWidget() override { dynamic_cast<WidgetInterface*>(getWidget())->updateWidget(); }
      QString getName() const { return comboBox->currentText(); }
      int getIndex() const { return comboBox->currentIndex(); }
      void setIndex(int i) { return comboBox->setCurrentIndex(i); }
      void resize_(int m, int n) override { dynamic_cast<WidgetInterface*>(getWidget())->resize_(m,n); }
      void setWidgetFactory(WidgetFactory *factory_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;

    public slots:
      void defineWidget(int);

    protected:
      QBoxLayout *layout;
      QComboBox *comboBox;
      QWidget *widget;
      WidgetFactory *factory;
      int mode;
  };

  class ContainerWidget : public Widget {

    public:
      ContainerWidget();

      void resize_(int m, int n) override;
      void addWidget(QWidget *widget_);
      QWidget* getWidget(int i) const {return widget[i];}
      void updateWidget() override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QVBoxLayout *layout;
      std::vector<QWidget*> widget;
  };

  class ListWidget : public Widget {
    Q_OBJECT

    public:
      ListWidget(WidgetFactory *factory, const QString &name="Element", int m=0, int mode=0, bool fixedSize=false, int minSize=0, int maxSize=100);
      ~ListWidget() override;
      void resize_(int m, int n) override;
      int getSize() const;
      void setSize(int m);
      QWidget* getWidget(int i) const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QStackedWidget *stackedWidget;
      QSpinBox* spinBox;
      QListWidget *list;
      WidgetFactory *factory;
      QString name;
      int mode;
      void addElements(int n=1, bool emitSignals=true);

    protected slots:
      void removeElements(int n=1);
      void changeCurrent(int idx);
      void currentIndexChanged(int idx);
  };

  class ChoiceWidgetFactory : public WidgetFactory {
    public:
      ChoiceWidgetFactory(WidgetFactory *factory_, int mode_=1) : factory(factory_), mode(mode_) { }
      Widget* createWidget(int i=0) override;
      MBXMLUtils::FQN getXMLName(int i=0) const override { return factory->getXMLName(i); }
    protected:
      WidgetFactory *factory;
      int mode;
  };

}

#endif
