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
      std::string oldID;
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
      std::string oldID;
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
      void setFrame(const QString &str, Frame *framePtr);

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
      void setFrame(const QString &str, Frame *framePtr);

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
      void setFrame(const QString &str, Frame *framePtr);
      void setDefaultFrame(const QString &def);
      QString getFrame() const;

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
      void setContour(const QString &str, Contour *contourPtr);
      QString getContour() const;

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
      void setBody(const QString &str, RigidBody *bodyPtr);
      QString getBody() const;
      RigidBody* getSelectedBody() {return selectedBody;}

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
      void setBody(const QString &str, RigidBody *bodyPtr);
      QString getBody() const;
      RigidBody* getSelectedBody() {return selectedBody;}
      QString getRatio() const {return ratio->text().isEmpty()?"0":ratio->text();}
      void setRatio(const QString &str) {ratio->setText(str=="0"?"":str);}

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
      void setObject(const QString &str, Object *objectPtr);
      QString getObject() const;
      Object* getSelectedObject() {return selectedObject;}

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
      void setLink(const QString &str, Link *linkPtr);
      QString getLink() const;
      Link* getSelectedLink() {return selectedLink;}

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
      void setConstraint(const QString &str, Constraint *constraintPtr);
      QString getConstraint() const;
      Constraint* getSelectedConstraint() {return selectedConstraint;}

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
      void setSignal(const QString &str, Signal *signalPtr);
      QString getSignal() const;
      Signal* getSelectedSignal() {return selectedSignal;}

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
      virtual QString getText() const = 0;
      virtual void setText(const QString &text) = 0;
  };

  class TextWidget : public BasicTextWidget {

    public:
      TextWidget(const QString &text="", bool readOnly=false);

      QString getText() const {return text->text();}
      void setText(const QString &text_) {text->setText(text_);}

    protected:
      QLineEdit *text;
  };

  class TextChoiceWidget : public BasicTextWidget {

    public:
      TextChoiceWidget(const std::vector<QString> &list, int num=0, bool editable=false);
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

    friend class ConnectFramesProperty;

    public:
    ConnectFramesWidget(int n, Element* element);

    void setDefaultFrame(const QString &def) { widget[0]->setDefaultFrame(def); }
    void updateWidget();

    protected:
    std::vector<FrameOfReferenceWidget*> widget;
    Element* element;
  };

  class ConnectContoursWidget : public Widget {

    friend class ConnectContoursProperty;

    public:
    ConnectContoursWidget(int n, Element* element);

    void updateWidget();

    protected:
    std::vector<ContourOfReferenceWidget*> widget;
    Element* element;
  };

  class ConnectRigidBodiesWidget : public Widget {

    friend class ConnectRigidBodiesProperty;

    public:
    ConnectRigidBodiesWidget(int n, Element* element);

//    void setDefaultRigidBody(const QString &def) { widget[0]->setDefaultRigidBody(def); }
    void updateWidget();
    RigidBodyOfReferenceWidget* getWidget(int i) { return widget[i]; }

    protected:
    std::vector<RigidBodyOfReferenceWidget*> widget;
    Element* element;
  };

  class DynamicSystemSolverTolerancesWidget : public Widget {
    friend class DynamicSystemSolverTolerancesProperty;

    public:
    DynamicSystemSolverTolerancesWidget();

    protected:
    ExtWidget *projection, *g, *gd, *gdd, *la, *La;
  };

  class DynamicSystemSolverParametersWidget : public Widget {

    friend class DynamicSystemSolverParametersProperty;

    public:
    DynamicSystemSolverParametersWidget();

    protected:
    ExtWidget *constraintSolver, *impactSolver, *numberOfMaximalIterations, *tolerances;
  };

  class PlotFeature : public Widget {
    public:
      PlotFeature(const QString &name);

    protected:
      QString name;
      QComboBox *status;
  };

  class EmbedWidget : public Widget {

    friend class EmbedProperty;

    public:
    EmbedWidget();

    protected:
    ExtWidget *href, *count, *counterName, *parameterList;

  };

  class SignalReferenceWidget : public Widget {

    friend class SignalReferenceProperty;

    public:
    SignalReferenceWidget(Element* element);
    Signal* getSelectedSignal() {return refSignal->getSelectedSignal();}
    SignalOfReferenceWidget* getSignalOfReferenceWidget() {return refSignal;}
    void updateWidget() {refSignal->updateWidget();}
    protected:
    SignalOfReferenceWidget* refSignal;
    ExtWidget *factor;
  };

  class ColorWidget : public Widget {
    Q_OBJECT

    friend class ColorProperty;

    public:
    ColorWidget();
    void updateWidget();

    protected:
    ExtWidget *color;
    QPushButton *button;

    protected slots:
      void setColor(); 
  };

  class PlotFeatureStatusWidget : public Widget {
    Q_OBJECT

    friend class PlotFeatureStatusProperty;

    protected:
      QComboBox *type, *value;
      QTreeWidget *tree;

    public:
      PlotFeatureStatusWidget(const std::vector<MBXMLUtils::FQN> &types);

    protected slots:
      void addFeature();
      void removeFeature();
      void updateFeature();
      void currentItemChanged(QTreeWidgetItem *item, QTreeWidgetItem *prev);
  };

}

#endif
