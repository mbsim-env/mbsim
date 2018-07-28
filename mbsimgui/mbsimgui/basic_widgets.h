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

#ifndef _BASIC_WIDGETS_H_
#define _BASIC_WIDGETS_H_

#include "element.h"
#include "extended_widgets.h"
#include "custom_widgets.h"
#include "utils.h"
#include "dialogs.h"
#include "namespace.h"
#include <QLineEdit>
#include <QTextEdit>
#include <QSpinBox>
#include <mbxmlutilshelper/dom.h>

class QPushButton;
class QComboBox;
class QCheckBox;
class QTreeWidget;
class QTreeWidgetItem;
 
namespace MBSimGUI {

  class RigidBody;
  class Frame;
  class Contour;
  class Parameter;
  class ExtWidget;

  class LocalFrameComboBox : public CustomComboBox {
    Q_OBJECT
    public:
      LocalFrameComboBox(Element *element_, QWidget *parent = nullptr);
    protected:
      Element *element;
      QString oldID;
      void showPopup() override;
      void hidePopup() override;
    protected slots:
      void highlightObject(const QString &frame);
  };

  class ParentFrameComboBox : public CustomComboBox {
    Q_OBJECT
    public:
      ParentFrameComboBox(Element *element_, QWidget *parent = nullptr);
    protected:
      Element *element;
      QString oldID;
      void showPopup() override;
      void hidePopup() override;
    protected slots:
      void highlightObject(const QString &frame);
  };

  class LocalFrameOfReferenceWidget : public Widget {
    Q_OBJECT

    public:
      LocalFrameOfReferenceWidget(Element* element_, Frame* omitFrame_=nullptr);

      void updateWidget() override;
      QString getFrame() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QComboBox *frame;
      Element* element;
      Frame *selectedFrame, *omitFrame;

    protected slots:
      void setFrame(const QString &str);
  };

  class ParentFrameOfReferenceWidget : public Widget {
    Q_OBJECT

    public:
      ParentFrameOfReferenceWidget(Element* element_, Frame* omitFrame_=nullptr);

      void updateWidget() override;
      QString getFrame() const; 
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QComboBox *frame;
      Element* element;
      Frame *selectedFrame, *omitFrame;

    protected slots:
      void setFrame(const QString &str);
  };

  class BasicElementOfReferenceWidget : public Widget {
    Q_OBJECT

    public:
      BasicElementOfReferenceWidget(Element* element_, Element* selectedElement_, BasicElementBrowser *eleBrowser_, bool addRatio);

      void setDefaultElement(const QString &def_);
      void setElement(const QString &str);
      QString getElement() const;
      void setRatio(const QString &str) { ratio->setText(str=="0"?"":str); }
      QString getRatio() const {return ratio->text().isEmpty()?"0":ratio->text();}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      virtual Element* findElement(const QString &str) { return nullptr; }
      QLineEdit *ele, *ratio;
      Element *element, *selectedElement;
      BasicElementBrowser *eleBrowser;
      QString def;

    public slots:
      void setElement();
  };

  template <class T>
  class ElementOfReferenceWidget : public BasicElementOfReferenceWidget {

    public:
      ElementOfReferenceWidget(Element* element, Element* selectedElement, bool addRatio=false) : BasicElementOfReferenceWidget(element,selectedElement,new ElementBrowser<T>(selectedElement),addRatio) {
      }
    protected:
      Element* findElement(const QString &str) override { return element->getByPath<T>(str); }
  };

  class FileWidget : public Widget {
    Q_OBJECT

    public:
      FileWidget(const QString &file, const QString &description_, const QString &extensions_, int mode_=0, bool quote_=false, bool absPath=true);
      QString getFile() const { return filePath->text(); }
      void setFile(const QString &str);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QLineEdit *filePath;
      QCheckBox *path;
      QString description, extensions;
      int mode;
      bool quote;

    protected slots:
      void selectFile();
      void changePath(int i);
  };

  class IntegerWidget : public Widget {

    public:
      virtual int getValue() = 0;
      virtual void setValue(int val) = 0;
  };

  class SpinBoxWidget : public IntegerWidget {
    Q_OBJECT

    public:
      SpinBoxWidget(int val=0, int min=0, int max=99);
      int getValue() override {return value->value();}
      void setValue(int val) override {value->setValue(val);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QSpinBox *value;

    signals:
      void valueChanged(int);
  };

  class ComboBoxWidget : public IntegerWidget {
    Q_OBJECT

    public:
      ComboBoxWidget(const QStringList &names, int currentIndex=0);
      int getValue() override {return value->currentIndex();}
      void setValue(int val) override {value->setCurrentIndex(val);}

    protected:
      QComboBox *value;

    signals:
      void valueChanged(int);
  };

  class BasicTextWidget : public Widget {

    public:
      BasicTextWidget() = default;
      virtual QString getText() const = 0;
      virtual void setText(const QString &text) = 0;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class TextWidget : public BasicTextWidget {

    public:
      TextWidget(const QString &text_="", bool readOnly=false);

      QString getText() const override {return text->text();}
      void setText(const QString &text_) override {text->setText(text_);}
      void setReadOnly(bool readOnly) { text->setReadOnly(readOnly); }

    protected:
      QLineEdit *text;
  };

  class TextChoiceWidget : public BasicTextWidget {

    public:
      TextChoiceWidget(const std::vector<QString> &list, int num=0, bool editable=false);
      QString getText() const override {return text->currentText();}
      void setText(const QString &str) override {
        if(text->isEditable())
          text->setEditText(str);
        else
          text->setCurrentIndex(text->findText(str));
      }

    protected:
      QComboBox *text;
  };

  class BasicConnectElementsWidget : public Widget {

    public:
      BasicConnectElementsWidget(const std::vector<BasicElementOfReferenceWidget*> widget_, const QString &name);

      void setDefaultElement(const QString &def_) { def = def_; widget[0]->setDefaultElement(def); }
      void updateWidget() override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      std::vector<BasicElementOfReferenceWidget*> widget;
      QString def;
  };

  template <class T>
  class ConnectElementsWidget : public BasicConnectElementsWidget {
    public:
      ConnectElementsWidget(int n, Element *element) : BasicConnectElementsWidget(create(n,element),T().getType()) { }
    protected:
      std::vector<BasicElementOfReferenceWidget*> create(int n, Element *element) {
        std::vector<BasicElementOfReferenceWidget*> widget(n);
        for(int i=0; i<n; i++) widget[i] = new ElementOfReferenceWidget<T>(element,nullptr);
        return widget;
      }
  };

  class ColorWidget : public Widget {
    Q_OBJECT

    public:
      ColorWidget(const std::vector<QString> &c=getBlueColor());
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      ExtWidget *color;
      QPushButton *button;

    protected slots:
      void setColor(); 
  };

  class PlotFeatureWidget : public Widget {
    Q_OBJECT

    protected:
      std::vector<MBXMLUtils::FQN> feature;
      QComboBox *type, *value, *nspace;
      ChoiceWidget2 *status;
      QTreeWidget *tree;
      MBXMLUtils::NamespaceURI uri;

    public:
      PlotFeatureWidget(const QString &types="", MBXMLUtils::NamespaceURI uri_=MBSIM);
      void addFeature(const MBXMLUtils::FQN &feature_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      xercesc::DOMElement* initializeUsingXML2(xercesc::DOMElement *parent);
      xercesc::DOMElement* writeXMLFile2(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr);

    protected slots:
      void updateNamespace(int i);
      void addFeature();
      void removeFeature();
      void updateFeature();
      void currentItemChanged(QTreeWidgetItem *item, QTreeWidgetItem *prev);
  };

  class XMLEditorWidget : public Widget {
    Q_OBJECT

    public:
      XMLEditorWidget();
//      int getValue() override {return value->value();}
//      void setValue(int val) override {value->setValue(val);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QTextEdit *edit;
//
//    signals:
//      void valueChanged(int);
  };

  template <class T>
  class ElementOfReferenceWidgetFactory : public WidgetFactory {
    public:
      ElementOfReferenceWidgetFactory(MBXMLUtils::FQN xmlName_, Element* element_, bool addRatio_=false) : xmlName(std::move(xmlName_)), element(element_), addRatio(addRatio_) { }
      QWidget* createWidget(int i=0) override { return new ElementOfReferenceWidget<T>(element,nullptr,addRatio); }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName; }
    protected:
      MBXMLUtils::FQN xmlName;
      Element *element;
      bool addRatio;
  };

}

#endif
