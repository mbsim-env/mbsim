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

#include "xml_widget.h"
#include "extended_widgets.h"
#include <QLineEdit>

class Element;
class RigidBody;
class Frame;
class Contour;
class Parameter;
class QComboBox;
class QStackedWidget;
class QListWidget;
class FrameBrowser;
class ContourBrowser;
class RigidBodyBrowser;
class ExtXMLWidget;
class QLabel;

class LocalFrameOfReferenceWidget : public XMLWidget {
  Q_OBJECT

  public:
    LocalFrameOfReferenceWidget(const std::string &xmlName, Element* element, Frame* omitFrame=0);

    void update();
    Frame* getFrame() {return selectedFrame;}
    void setFrame(Frame* frame_);
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    QComboBox *frame;
    Element* element;
    Frame *selectedFrame, *omitFrame;
    std::string xmlName;

  protected slots:
    void setFrame(const QString &str);
};

class FrameOfReferenceWidget : public XMLWidget {
  Q_OBJECT

  public:
    FrameOfReferenceWidget(const std::string &xmlName, Element* element, Frame* selectedFrame);

    void initialize();
    void update();
    Frame* getFrame() {return selectedFrame;}
    void setFrame(Frame* frame_);
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void setSavedFrameOfReference(const QString &str) {saved_frameOfReference = str;}
    const QString& getSavedFrameOfReference() const {return saved_frameOfReference;}

  protected:
    QLineEdit *frame;
    Element* element;
    FrameBrowser *frameBrowser;
    Frame *selectedFrame;
    std::string xmlName;
    QString saved_frameOfReference;

  public slots:
    void setFrame();
};

class ContourOfReferenceWidget : public XMLWidget {
  Q_OBJECT

  public:
    ContourOfReferenceWidget(const std::string &xmlName, Element* element, Contour* selectedContour);

    void initialize();
    void update();
    Contour* getContour() {return selectedContour;}
    void setContour(Contour* contour_);
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void setSavedContourOfReference(const QString &str) {saved_contourOfReference = str;}
    const QString& getSavedContourOfReference() const {return saved_contourOfReference;}

  protected:
    QLineEdit *contour;
    Element* element;
    ContourBrowser *contourBrowser;
    Contour *selectedContour;
    std::string xmlName;
    QString saved_contourOfReference;

  public slots:
    void setContour();
};

class RigidBodyOfReferenceWidget : public XMLWidget {
  Q_OBJECT

  public:
    RigidBodyOfReferenceWidget(const std::string &xmlName, Element* element, RigidBody* selectedBody);

    void initialize();
    void update();
    RigidBody* getBody() {return selectedBody;}
    void setBody(RigidBody* body_);
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void setSavedBodyOfReference(const QString &str) {saved_bodyOfReference = str;}
    const QString& getSavedBodyOfReference() const {return saved_bodyOfReference;}

  protected:
    QLineEdit* body;
    Element* element;
    RigidBodyBrowser* bodyBrowser;
    RigidBody* selectedBody;
    std::string xmlName;
    QString saved_bodyOfReference;

  public slots:
    void setBody();

  signals:
    void bodyChanged();
};

class FileWidget : public XMLWidget {
  Q_OBJECT

  public:
    FileWidget(const std::string &xmlName, const QString &description, const QString &extensions);
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    QLineEdit *fileName;
    std::string xmlName;
    QString description, extensions;

  protected slots:
    void selectFile();

};

class NameWidget : public XMLWidget {
  Q_OBJECT

  public:
    NameWidget(Element* ele, bool renaming=true);

    QString getName() const {return ename->text();}
    void setName(const QString &name) {ename->setText(name);}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    QLineEdit *ename;
    Element* element;

  protected slots:
    void rename();
};

class ElementPositionWidget2 : public XMLWidget {

  public:
    ElementPositionWidget2(Element *element);

    void update() {refFrame->update();}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    Element *getElement() {return element;}

  protected:
    Element *element;
    ExtPhysicalVarWidget *position, *orientation;
    FrameOfReferenceWidget *refFrame;
};

class ElementPositionWidget : public XMLWidget {

  public:
    ElementPositionWidget(Element *element);

    void update() {refFrame->update();}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    Element *getElement() {return element;}

  protected:
    Element *element;
    ExtPhysicalVarWidget *position, *orientation;
    LocalFrameOfReferenceWidget *refFrame;
};

class FramePositionsWidget : public XMLWidget {
  Q_OBJECT

  public:
    FramePositionsWidget(Element *element);

    void update();
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    Element *element;
    QStackedWidget *stackedWidget; 
    QListWidget *frameList; 
  protected slots:
    void changeCurrent(int idx);
};

class ContourPositionsWidget : public XMLWidget {
  Q_OBJECT

  public:
    ContourPositionsWidget(Element *element);

    void update();
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    Element *element;
    QStackedWidget *stackedWidget; 
    QListWidget *contourList; 
  protected slots:
    void changeCurrent(int idx);
};

class ConnectFramesWidget : public XMLWidget {

  public:
    ConnectFramesWidget(int n, Element* element);

    void initialize();
    void update();
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    std::vector<FrameOfReferenceWidget*> widget;
    Element* element;
};

class ConnectContoursWidget : public XMLWidget {

  public:
    ConnectContoursWidget(int n, Element* element);

    void initialize();
    void update();
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    std::vector<ContourOfReferenceWidget*> widget;
    Element* element;
};

class DependenciesWidget : public XMLWidget {
  Q_OBJECT

  public:
    DependenciesWidget(const std::string &xmlName, Element* element);

    void update(); 
    void initialize();
    RigidBody* getBody(int i) {return refBody[i]->getBody();}
    void addBody(int i, RigidBody* body_);
    int getSize() const {return refBody.size();}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    Element* element;
    std::string xmlName;
    std::vector<RigidBody*> selectedBody;
    std::vector<RigidBodyOfReferenceWidget*> refBody;
    QStackedWidget *stackedWidget; 
    QListWidget *bodyList; 

  protected slots:
    void updateList();
    void addDependency();
    void removeDependency();
    void updateGeneralizedCoordinatesOfBodies();
    void openContextMenu(const QPoint &pos);

  signals:
    void bodyChanged();
};

class ParameterNameWidget : public XMLWidget {
  Q_OBJECT

  public:
    ParameterNameWidget(Parameter* ele, bool renaming=true);

    QString getName() const {return ename->text();}
    void setName(const QString &name) {ename->setText(name);}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    QLineEdit *ename;
    Parameter* parameter;

  protected slots:
    void rename();
};

class ParameterValueWidget : public XMLWidget {
  Q_OBJECT

  public:
    ParameterValueWidget(PhysicalStringWidget *var);

    ExtPhysicalVarWidget* getExtPhysicalWidget() {return widget;}
    virtual std::string getValue() const { return widget->getValue(); }
    virtual bool initializeUsingXML(TiXmlElement *element) {return true;}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) {return 0;}

  protected:
    ExtPhysicalVarWidget *widget;
  protected slots:
    void parameterChanged();
  signals:
    void parameterChanged(const QString &str);
};

class SolverTolerances : public XMLWidget {

  public:
    SolverTolerances();

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    ExtXMLWidget *projection, *g, *gd, *gdd, *la, *La;
};

class SolverParameters : public XMLWidget {

  public:
    SolverParameters();

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    ExtXMLWidget *tolerances;
};

class PlotFeature : public XMLWidget {
  public:
    PlotFeature(const std::string &name);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
  protected:
    std::string name;
    QComboBox *status;
};

#endif
