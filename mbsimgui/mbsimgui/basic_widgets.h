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
#include <QFileDialog>
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

    public:
      LocalFrameComboBox(Element *element_, QWidget *parent = nullptr);
    protected:
      void showPopup() override;
      void hidePopup() override;
      void highlightObject(const QString &frame);
      Element *element;
      std::string oldID;
  };

  class ParentFrameComboBox : public CustomComboBox {

    public:
      ParentFrameComboBox(Element *element_, QWidget *parent = nullptr);
    protected:
      void showPopup() override;
      void hidePopup() override;
      void highlightObject(const QString &frame);
      Element *element;
      std::string oldID;
  };

  class LocalFrameOfReferenceWidget : public Widget {


    public:
      LocalFrameOfReferenceWidget(Element* element_, Frame* omitFrame_=nullptr);

      void updateWidget() override;
      QString getFrame() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      void setFrame(const QString &str);
      QComboBox *frame;
      Element* element;
      Frame *selectedFrame, *omitFrame;
  };

  class ParentFrameOfReferenceWidget : public Widget {

    public:
      ParentFrameOfReferenceWidget(Element* element_, Frame* omitFrame_=nullptr);

      void updateWidget() override;
      QString getFrame() const; 
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      void setFrame(const QString &str);
      QComboBox *frame;
      Element* element;
      Frame *selectedFrame, *omitFrame;
  };

  class BasicElementOfReferenceWidget : public Widget {

    public:
      BasicElementOfReferenceWidget(Element* element_, Element* selectedElement, BasicElementBrowser *eleBrowser_, bool addRatio);

      void setDefaultElement(const QString &def) { ele->setPlaceholderText(def); }
      void setElement(const QString &str) { if(str!=ele->placeholderText()) ele->setText(str); }
      void clear() { ele->setText(""); }
      QString getElement() const { return ele->text().isEmpty()?ele->placeholderText():ele->text(); }
      void setRatio(const QString &str) { ratio->setText(str=="0"?"":str); }
      QString getRatio() const { return ratio->text().isEmpty()?"0":ratio->text(); }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      virtual Element* findElement(const QString &str) { return nullptr; }
      void setElement();
      void showBrowser();
      QLineEdit *ele, *ratio;
      Element *element;
      BasicElementBrowser *eleBrowser;
  };

  template <class T>
  class ElementOfReferenceWidget : public BasicElementOfReferenceWidget {

    public:
      ElementOfReferenceWidget(Element* element, Element* selectedElement, QWidget *parent) : BasicElementOfReferenceWidget(element,selectedElement,new ElementBrowser<T>(selectedElement,parent),false) { }
      ElementOfReferenceWidget(Element* element, Element* selectedElement, bool addRatio, QWidget *parent) : BasicElementOfReferenceWidget(element,selectedElement,new ElementBrowser<T>(selectedElement,parent),addRatio) { }
    protected:
      Element* findElement(const QString &str) override { return element->getByPath<T>(str); }
  };

  class FileWidget : public Widget {
    Q_OBJECT

    public:
      FileWidget(const QString &file, const QString &description_, const QString &extensions_, int mode_=0, bool quote_=false, bool absPath=true, QFileDialog::Options options_=QFileDialog::Options());
      QString getFile() const { return filePath->text(); }
      void setFile(const QString &str);
      bool getAbsolutePath() const { return path->isChecked(); }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      void selectFile();
      void changePath(int i);
      QLineEdit *filePath;
      QCheckBox *path;
      QString description, extensions;
      int mode;
      bool quote;
      QFileDialog::Options options;

    signals:
      void valueChanged(const QString&);
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
      int getValue() override { return value->value(); }
      void setValue(int val) override {value->setValue(val);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QSpinBox *value;

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

      QString getText() const override { return text->text(); }
      void setText(const QString &text_) override {text->setText(text_);}
      void setReadOnly(bool readOnly) { text->setReadOnly(readOnly); }

    protected:
      QLineEdit *text;
  };

  class TextChoiceWidget : public BasicTextWidget {

    public:
      TextChoiceWidget(const std::vector<QString> &list, int num=0, bool editable=false);
      QString getText() const override { return text->currentText(); }
      void setStringList(const std::vector<QString> &list);
      void setText(const QString &str) override {
        if(text->isEditable())
          text->setEditText(str);
        else
          text->setCurrentIndex(text->findText(str));
      }
      void setCurrentIndex(int num);

    protected:
      QComboBox *text;
  };

  class BasicConnectElementsWidget : public Widget {

    public:
      BasicConnectElementsWidget(const std::vector<BasicElementOfReferenceWidget*> widget_, const std::vector<QString> &name);

      void setDefaultElement(const QString &def_) { def = def_; widget[0]->setDefaultElement(def); }
      void updateWidget() override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      std::vector<BasicElementOfReferenceWidget*> widget;
      QString def;
  };

  template <class T1, class T2=T1>
  class ConnectElementsWidget : public BasicConnectElementsWidget {
    public:
      ConnectElementsWidget(int n, Element *element, QWidget *parent) : BasicConnectElementsWidget(create(n,element,parent),create(n)) { }
    protected:
      std::vector<BasicElementOfReferenceWidget*> create(int n, Element *element, QWidget *parent) {
        std::vector<BasicElementOfReferenceWidget*> widget(n);
        widget[0] = new ElementOfReferenceWidget<T1>(element,nullptr,parent);
        if(n>1)
          widget[1] = new ElementOfReferenceWidget<T2>(element,nullptr,parent);
        return widget;
      }
      std::vector<QString> create(int n) {
        std::vector<QString> name(n);
        name[0] = T1::getTypeStatic();
        if(n>1)
          name[1] = T2::getTypeStatic();
        return name;
      }
  };

  class ColorWidget : public Widget {

    public:
      ColorWidget(const std::vector<QString> &c=getBlueColor());
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      void setColor();
      ExtWidget *color;
      QPushButton *button;
  };

  class PlotFeatureWidget : public Widget {

    protected:
      void updateNamespace(int i);
      void addFeature();
      void removeFeature();
      void updateFeature();
      void currentItemChanged(QTreeWidgetItem *item, QTreeWidgetItem *prev);
      std::vector<MBXMLUtils::FQN> feature;
      QComboBox *type, *value, *nspace;
      ChoiceWidget *status;
      QTreeWidget *tree;
      MBXMLUtils::NamespaceURI uri;

    public:
      PlotFeatureWidget(const QString &types="", MBXMLUtils::NamespaceURI uri_=MBSIM);
      void addFeature(const MBXMLUtils::FQN &feature_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      xercesc::DOMElement* initializeUsingXML2(xercesc::DOMElement *parent);
      xercesc::DOMElement* writeXMLFile2(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr);
  };

  class XMLEditorWidget : public Widget {

    public:
      XMLEditorWidget(const QString &text="");
      QString getText() { return edit->toPlainText(); }
      void setText(const QString &text) { edit->setPlainText(text); }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QTextEdit *edit;
  };

  class CloneWidget : public Widget {

    public:
      CloneWidget();
      void setCount(const QString &count_);
      QString getCount() const;
      void setCounterName(const QString &counterName_);
      QString getCounterName() const;

    protected:
      ExtWidget *count, *counterName;
  };

  template <class T>
  class ElementOfReferenceWidgetFactory : public WidgetFactory {
    public:
      ElementOfReferenceWidgetFactory(MBXMLUtils::FQN xmlName_, Element* element_, QWidget *parent_) : xmlName(std::move(xmlName_)), element(element_), addRatio(false), parent(parent_) { }
      ElementOfReferenceWidgetFactory(MBXMLUtils::FQN xmlName_, Element* element_, bool addRatio_, QWidget *parent_) : xmlName(std::move(xmlName_)), element(element_), addRatio(addRatio_), parent(parent_) { }
      Widget* createWidget(int i=0) override { return new ElementOfReferenceWidget<T>(element,nullptr,addRatio,parent); }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName; }
    protected:
      MBXMLUtils::FQN xmlName;
      Element *element;
      bool addRatio;
      QWidget *parent;
  };

  class ExtStringWidget : public Widget {
   public:
      ExtStringWidget(Element *element_);
      void showBrowser();
      void setElement();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override { return value->initializeUsingXML(element); }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override { return value->writeXMLFile(parent,ref); }

    protected:
      Element *element;
      ChoiceWidget *value;
      BasicElementBrowser *eleBrowser;
  };

  class StateWidget : public Widget {
    protected:
      void addState();
      void removeState();
      void updateState();
      void clear();
      void currentItemChanged(QTreeWidgetItem *item, QTreeWidgetItem *prev);
      ChoiceWidget *name;
      ChoiceWidget *value;
      QTreeWidget *tree;

    public:
      StateWidget();
      std::vector<QString> getNames() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class TransitionWidget : public Widget {
    protected:
      void addTransition();
      void removeTransition();
      void updateTransition();
      void clear();
      void currentItemChanged(QTreeWidgetItem *item, QTreeWidgetItem *prev);
      Element *element;
      TextChoiceWidget *src;
      TextChoiceWidget *dest;
      BasicElementOfReferenceWidget *sig;
      ChoiceWidget *th;
      QTreeWidget *tree;

    public:
      TransitionWidget(Element *element_);
      void setStringList(const std::vector<QString> &list) { src->setStringList(list); dest->setStringList(list); }
      TextChoiceWidget *getDestChoice() { return dest; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

}

#endif
