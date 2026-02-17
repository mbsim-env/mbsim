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
#include <QPlainTextEdit>
#include <QTextEdit>
#include <QSpinBox>
#include <QFileDialog>
#include <mbxmlutilshelper/dom.h>

class QPushButton;
class QComboBox;
class QCheckBox;
class QTreeWidget;
class QTreeWidgetItem;
class QStackedWidget;
 
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
      void setFrame(int index);
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
      void setFrame(int index);
      QComboBox *frame;
      Element* element;
      Frame *selectedFrame, *omitFrame;
  };

  class BasicElementOfReferenceWidget : public Widget {

    public:
      BasicElementOfReferenceWidget(Element* element_, Element* selectedElement, BasicElementBrowser *eleBrowser_, bool addRatio);

      void setDefaultElement(const QString &def) { ele->setPlaceholderText(def); }
      void setElement(const QString &str) { ele->setText(str); }
      void clear() { ele->setText(""); }
      QString getElement() const { return ele->text().isEmpty()?ele->placeholderText():ele->text(); }
      void setRatio(const QString &str) { ratio->setText(str); }
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

  class EmbedableListWidget : public Widget {
    protected:
      MBXMLUtils::FQN xmlName;
      QTreeWidget *tree;
      CustomSpinBox *spinBox;
      bool allowEmbed;
    public:
      EmbedableListWidget(QWidget *parent, const MBXMLUtils::FQN &xmlName_, bool allowEmbed_) : Widget(parent), xmlName(xmlName_), allowEmbed(allowEmbed_) { }
      void itemDoubleClicked(QTreeWidgetItem *item, int column);
      QString getXMLComment(xercesc::DOMElement *element) override;
      void setXMLComment(const QString &comment, xercesc::DOMNode *element) override;
      int getSize() const;
  };

  class BasicElementsOfReferenceWidget : public EmbedableListWidget {
    protected:
      virtual Element* findElement(const QString &str) { return nullptr; }
      void updateTreeItem();
      void addElement();
      void removeElement();
      void showBrowser();
      void convertToArrayPattern();
      void changeNumberOfElements(int num);
      void openMenu();
      QPushButton *add, *remove;
      Element *element;
      BasicElementBrowser *eleBrowser;

    public:
      BasicElementsOfReferenceWidget(MBXMLUtils::FQN xmlName_, Element *element_, BasicElementBrowser *eleBrowser_, int min, int max, bool addRatio, bool allowEmbed_);
      void setRange(int min, int max);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  template <class T>
  class ElementsOfReferenceWidget : public BasicElementsOfReferenceWidget {

    public:
      ElementsOfReferenceWidget(MBXMLUtils::FQN xmlName, Element* element, int min, int max, QWidget *parent, bool allowEmbed=false) : BasicElementsOfReferenceWidget(xmlName,element,new ElementBrowser<T>(nullptr,parent),min,max,false,allowEmbed) { }
      ElementsOfReferenceWidget(MBXMLUtils::FQN xmlName, Element* element, int min, int max, bool addRatio, QWidget *parent, bool allowEmbed=false) : BasicElementsOfReferenceWidget(xmlName,element,new ElementBrowser<T>(nullptr,parent),min,max,addRatio,allowEmbed) { }
    protected:
      Element* findElement(const QString &str) override { return element->getByPath<T>(str); }
  };

  class FileWidget : public Widget {
    Q_OBJECT

    public:
      FileWidget(const QString &file, const QString &description_, const QString &extensions_, int mode_=0, bool quote_=false, bool absPath=true, QFileDialog::Options options_=QFileDialog::Options());
      QString getFile(bool rmQuote=false) const { return (quote and rmQuote)?filePath->text().mid(1,filePath->text().length()-2):filePath->text(); }
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

    Q_SIGNALS:
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

    Q_SIGNALS:
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
      void setText(const QString &text_) override { text->setText(text_); }
      void setReadOnly(bool readOnly) { text->setReadOnly(readOnly); }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;

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
      int getCurrentIndex() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      QComboBox *getWidget() { return text; }

    protected:
      QComboBox *text;
  };

  class TextEditorWidget : public BasicTextWidget {

    public:
      TextEditorWidget(const QString &text_="", bool readOnly=false);

      QString getText() const override { return text->toPlainText(); }
      void setText(const QString &text_) override { text->setPlainText(text_); }
      void setReadOnly(bool readOnly) { text->setReadOnly(readOnly); }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      void enableMonospaceFont();
      void enableSyntaxHighlighter();

    protected:
      QTextEdit *text;
  };

  class TextListWidget : public EmbedableListWidget {
    protected:
      void updateTreeItem();
      void addItem();
      void editItem();
      void removeItem();
      void changeNumberOfItems(int num);
      void openMenu();
      QString label;
      LineEditDialog *dialog;
      void convertToArrayPattern();

    public:
      TextListWidget(const QString &label_, const MBXMLUtils::FQN &xmlName_, bool allowEmbed_=false);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class BasicConnectElementsWidget : public Widget {

    public:
      BasicConnectElementsWidget(const std::vector<BasicElementOfReferenceWidget*> widget_, const std::vector<QString> &name);

      void setDefaultElement(const QString &def_, int i=0) { def[i] = def_; widget[i]->setDefaultElement(def_); }
      void updateWidget() override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      std::vector<BasicElementOfReferenceWidget*> widget;
      std::vector<QString> def;
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
      void updateTreeItem();
      void addFeature();
      void editFeature();
      void removeFeature();
      void changeNumberOfPlotFeatures(int num);
      void openMenu();
      QTreeWidget *tree;
      PlotFeatureDialog *dialog;
      CustomSpinBox *spinBox;
      MBXMLUtils::FQN specialType;

    public:
      PlotFeatureWidget(const MBXMLUtils::FQN &specialType_="");
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
      QTextEdit *getEditor() { return edit; }

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
      void setOnlyif(const QString &onlyif_);
      QString getOnlyif() const;

    protected:
      ExtWidget *count, *counterName, *onlyif;
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
      void updateTreeItem();
      void addState();
      void editState();
      void removeState();
      void changeNumberOfStates(int num);
      void openMenu();
      QTreeWidget *tree;
      StateDialog *dialog;
      CustomSpinBox *spinBox;

    public:
      StateWidget();
      std::vector<QString> getNames() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      QString getXMLComment(xercesc::DOMElement *element) override;
      void setXMLComment(const QString &comment, xercesc::DOMNode *element) override;
  };

  class TransitionWidget : public Widget {
    protected:
      void updateTreeItem();
      void addTransition();
      void editTransition();
      void removeTransition();
      void changeNumberOfTransitions(int num);
      void openMenu();
      QTreeWidget *tree;
      TransitionDialog *dialog;
      CustomSpinBox *spinBox;
      QString state1, state2;

    public:
      TransitionWidget(Element *element);
      void setStringList(const std::vector<QString> &list);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      QString getXMLComment(xercesc::DOMElement *element) override;
      void setXMLComment(const QString &comment, xercesc::DOMNode *element) override;
  };

  class CommentWidget : public Widget {

    public:
      CommentWidget();
      void setComment(const QString &comment);
      QString getComment() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QPlainTextEdit *edit;
  };

  class MBSimGUIContextAction : public Widget {

    public:
      MBSimGUIContextAction();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      int getStretchHint() const override { return 10000; }

    protected:
      QListWidget *name;
      QSpinBox *count;
      QStackedWidget *code;
  };

}

#endif
