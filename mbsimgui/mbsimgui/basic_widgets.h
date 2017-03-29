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

#include "extended_widgets.h"
#include "custom_widgets.h"
#include "property.h"
#include <QLineEdit>
#include <QSpinBox>
#include <mbxmlutilshelper/dom.h>

class QPushButton;
class QComboBox;
class QStackedWidget;
class QListWidget;
class QTreeWidget;
class QTreeWidgetItem;
 
namespace MBSimGUI {

  class Element;
  class Object;
  class Link;
  class Constraint;
  class RigidBody;
  class Frame;
  class Contour;
  class Parameter;
  class Signal;
  class FrameBrowser;
  class ContourBrowser;
  class RigidBodyBrowser;
  class ObjectBrowser;
  class LinkBrowser;
  class ConstraintBrowser;
  class SignalBrowser;
  class ExtWidget;

  class LocalFrameComboBox : public CustomComboBox {
    Q_OBJECT
    public:
      LocalFrameComboBox(Element *element, QWidget *parent = 0);
    protected:
      Element *element;
      QString oldID;
      virtual void showPopup();
      virtual void hidePopup();
    protected slots:
      void highlightObject(const QString &str);
  };

  class ParentFrameComboBox : public CustomComboBox {
    Q_OBJECT
    public:
      ParentFrameComboBox(Element *element, QWidget *parent = 0);
    protected:
      Element *element;
      QString oldID;
      virtual void showPopup();
      virtual void hidePopup();
    protected slots:
      void highlightObject(const QString &str);
  };

  class LocalFrameOfReferenceWidget : public Widget {
    Q_OBJECT

    public:
      LocalFrameOfReferenceWidget(Element* element, Frame* omitFrame=0);

      void updateWidget();
      QString getFrame() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

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
      ParentFrameOfReferenceWidget(Element* element, Frame* omitFrame=0);

      void updateWidget();
      QString getFrame() const; 
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      QComboBox *frame;
      Element* element;
      Frame *selectedFrame, *omitFrame;

    protected slots:
      void setFrame(const QString &str);
  };

  class FrameOfReferenceWidget : public Widget {
    Q_OBJECT

    public:
      FrameOfReferenceWidget(Element* element, Frame* selectedFrame);

      void updateWidget();
      void setFrame(const QString &str);
      void setDefaultFrame(const QString &def);
      QString getFrame() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      QLineEdit *frame;
      Element* element;
      FrameBrowser *frameBrowser;
      Frame *selectedFrame;
      QString def;

    public slots:
      void setFrame(); 
  };

  class ContourOfReferenceWidget : public Widget {
    Q_OBJECT

    public:
      ContourOfReferenceWidget(Element* element, Contour* selectedContour);

      void updateWidget();
      void setContour(const QString &str);
      QString getContour() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      QLineEdit *contour;
      Element* element;
      ContourBrowser *contourBrowser;
      Contour *selectedContour;

    public slots:
      void setContour();
  };

  class RigidBodyOfReferenceWidget : public Widget {
    Q_OBJECT

    public:
      RigidBodyOfReferenceWidget(Element* element, RigidBody* selectedBody);

      void updateWidget();
      void setBody(const QString &str);
      QString getBody() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      QLineEdit* body;
      Element* element;
      RigidBodyBrowser* bodyBrowser;
      RigidBody* selectedBody;

    public slots:
      void setBody();

    signals:
      void bodyChanged();
  };

  class GearInputReferenceWidget : public Widget {
    Q_OBJECT

    public:
      GearInputReferenceWidget(Element* element, RigidBody* selectedBody);

      void updateWidget();
      void setBody(const QString &str);
      QString getBody() const;
      QString getRatio() const {return ratio->text().isEmpty()?"0":ratio->text();}
      void setRatio(const QString &str) {ratio->setText(str=="0"?"":str);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      QLineEdit* body, *ratio;
      Element* element;
      RigidBodyBrowser* bodyBrowser;
      RigidBody* selectedBody;

    public slots:
      void setBody();

    signals:
      void bodyChanged();
  };

  class ObjectOfReferenceWidget : public Widget {
    Q_OBJECT

    public:
      ObjectOfReferenceWidget(Element* element, Object* selectedObject);

      void updateWidget();
      void setObject(const QString &str);
      QString getObject() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      QLineEdit* object;
      Element* element;
      ObjectBrowser* objectBrowser;
      Object* selectedObject;

    public slots:
      void setObject();

    signals:
      void objectChanged();
  };

  class LinkOfReferenceWidget : public Widget {
    Q_OBJECT

    public:
      LinkOfReferenceWidget(Element* element, Link* selectedLink);

      void updateWidget();
      void setLink(const QString &str);
      QString getLink() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      QLineEdit* link;
      Element* element;
      LinkBrowser* linkBrowser;
      Link* selectedLink;

    public slots:
      void setLink();

    signals:
      void linkChanged();
  };

  class ConstraintOfReferenceWidget : public Widget {
    Q_OBJECT

    public:
      ConstraintOfReferenceWidget(Element* element, Constraint* selectedConstraint);

      void updateWidget();
      void setConstraint(const QString &str);
      QString getConstraint() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      QLineEdit* constraint;
      Element* element;
      ConstraintBrowser* constraintBrowser;
      Constraint* selectedConstraint;

    public slots:
      void setConstraint();

    signals:
      void constraintChanged();
  };

  class SignalOfReferenceWidget : public Widget {
    Q_OBJECT

    public:
      SignalOfReferenceWidget(Element* element, Signal* selectedSignal);

      void updateWidget();
      void setSignal(const QString &str);
      QString getSignal() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      QLineEdit* signal;
      Element* element;
      SignalBrowser* signalBrowser;
      Signal* selectedSignal;

    public slots:
      void setSignal();

    signals:
      void signalChanged();
  };

  class FileWidget : public Widget {
    Q_OBJECT

    public:
      FileWidget(const QString &description, const QString &extensions, int mode=0);
      QString getFile() const { return relativeFilePath->text(); }
      void setFile(const QString &str) { relativeFilePath->setText(str); }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      QLineEdit *relativeFilePath;
      QString description, extensions;
      int mode;

    protected slots:
      void selectFile();

    signals:
      void fileChanged(const QString &str);
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
      int getValue() {return value->value();}
      void setValue(int val) {value->setValue(val);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      QSpinBox *value;

    signals:
      void valueChanged(int);
  };

  class ComboBoxWidget : public IntegerWidget {
    Q_OBJECT

    public:
      ComboBoxWidget(const QStringList &names, int currentIndex=0);
      int getValue() {return value->currentIndex();}
      void setValue(int val) {value->setCurrentIndex(val);}

    protected:
      QComboBox *value;

    signals:
      void valueChanged(int);
  };

  class BasicTextWidget : public Widget {

    public:
      BasicTextWidget(bool quote_=false) : quote(quote_) { }
      virtual QString getText() const = 0;
      virtual void setText(const QString &text) = 0;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      bool quote;
  };

  class TextWidget : public BasicTextWidget {

    public:
      TextWidget(const QString &text="", bool readOnly=false, bool quote=false);

      QString getText() const {return text->text();}
      void setText(const QString &text_) {text->setText(text_);}
      void setReadOnly(bool readOnly) { text->setReadOnly(readOnly); }

    protected:
      QLineEdit *text;
  };

  class TextChoiceWidget : public BasicTextWidget {

    public:
      TextChoiceWidget(const std::vector<QString> &list, int num=0, bool editable=false, bool quote=false);
      QString getText() const {return text->currentText();}
      void setText(const QString &str) {
        if(text->isEditable())
          text->setEditText(str);
        else
          text->setCurrentIndex(text->findText(str));
      }

    protected:
      QComboBox *text;
  };

  class ConnectFramesWidget : public Widget {

    public:
      ConnectFramesWidget(int n, Element* element);

      void setDefaultFrame(const QString &def_) { def = def_; widget[0]->setDefaultFrame(def); }
      void updateWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      std::vector<FrameOfReferenceWidget*> widget;
      QString def;
      Element* element;
  };

  class ConnectContoursWidget : public Widget {

    public:
      ConnectContoursWidget(int n, Element* element);

      void updateWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      std::vector<ContourOfReferenceWidget*> widget;
      Element* element;
  };

  class ConnectRigidBodiesWidget : public Widget {

    public:
      ConnectRigidBodiesWidget(int n, Element* element);

      void updateWidget();
      RigidBodyOfReferenceWidget* getWidget(int i) { return widget[i]; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      std::vector<RigidBodyOfReferenceWidget*> widget;
      Element* element;
  };

  class DynamicSystemSolverTolerancesWidget : public Widget {

    public:
      DynamicSystemSolverTolerancesWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      ExtWidget *projection, *g, *gd, *gdd, *la, *La;
  };

  class DynamicSystemSolverParametersWidget : public Widget {

    public:
      DynamicSystemSolverParametersWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      ExtWidget *constraintSolver, *impactSolver, *numberOfMaximalIterations, *tolerances;
  };

  class EmbedWidget : public Widget {

    public:
      EmbedWidget();
      QString getCounterName() const;
      QString getCount() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      ExtWidget *href, *count, *counterName, *parameterList;
  };

//  class SignalReferenceWidget : public Widget {
//
//    public:
//    SignalReferenceWidget(Element* element);
//    SignalOfReferenceWidget* getSignalOfReferenceWidget() {return refSignal;}
//    void updateWidget() {refSignal->updateWidget();}
//    protected:
//    SignalOfReferenceWidget* refSignal;
//    ExtWidget *factor;
//  };

  class ColorWidget : public Widget {
    Q_OBJECT

    public:
      ColorWidget();
      void updateWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      ExtWidget *color;
      QPushButton *button;

    protected slots:
      void setColor(); 
  };

  class PlotFeatureStatusWidget : public Widget {
    Q_OBJECT

    protected:
      QComboBox *type, *value, *status;
      QTreeWidget *tree;
      MBXMLUtils::NamespaceURI uri;

    public:
      PlotFeatureStatusWidget(const QString &types="", const MBXMLUtils::NamespaceURI &uri=MBSIM);
      void addFeature(const QString &feature);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
      xercesc::DOMElement* initializeUsingXML2(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile2(xercesc::DOMNode *element);

    protected slots:
      void addFeature();
      void removeFeature();
      void updateFeature();
      void currentItemChanged(QTreeWidgetItem *item, QTreeWidgetItem *prev);
  };

}

#endif
