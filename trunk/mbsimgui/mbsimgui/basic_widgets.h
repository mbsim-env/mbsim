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
class ExtWidget;
class QLabel;

class LocalFrameOfReferenceWidget : public Widget {
  Q_OBJECT

  public:
    LocalFrameOfReferenceWidget(Element* element, Frame* omitFrame=0);

    void updateWidget();
    Frame* getFrame() {return selectedFrame;}
    void setFrame(Frame* frame_);

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
    Frame* getFrame() {return selectedFrame;}
    void setFrame(Frame* frame_);
    void setSavedFrameOfReference(const QString &str) {saved_frameOfReference = str;}
    const QString& getSavedFrameOfReference() const {return saved_frameOfReference;}

  protected:
    QComboBox *frame;
    Element* element;
    Frame *selectedFrame, *omitFrame;
    QString saved_frameOfReference;

  protected slots:
    void setFrame(const QString &str);
};

class FrameOfReferenceWidget : public Widget {
  Q_OBJECT

  public:
    FrameOfReferenceWidget(Element* element, Frame* selectedFrame);

    void updateWidget();
    Frame* getFrame() {return selectedFrame;}
    void setFrame(Frame* frame_);
    void setSavedFrameOfReference(const QString &str) {saved_frameOfReference = str;}
    const QString& getSavedFrameOfReference() const {return saved_frameOfReference;}

  protected:
    QLineEdit *frame;
    Element* element;
    FrameBrowser *frameBrowser;
    Frame *selectedFrame;
    QString saved_frameOfReference;

  public slots:
    void setFrame();
};

class ContourOfReferenceWidget : public Widget {
  Q_OBJECT

  public:
    ContourOfReferenceWidget(Element* element, Contour* selectedContour);

    void updateWidget();
    Contour* getContour() {return selectedContour;}
    void setContour(Contour* contour_);
    void setSavedContourOfReference(const QString &str) {saved_contourOfReference = str;}
    const QString& getSavedContourOfReference() const {return saved_contourOfReference;}

  protected:
    QLineEdit *contour;
    Element* element;
    ContourBrowser *contourBrowser;
    Contour *selectedContour;
    QString saved_contourOfReference;

  public slots:
    void setContour();
};

class RigidBodyOfReferenceWidget : public Widget {
  Q_OBJECT

  public:
    RigidBodyOfReferenceWidget(Element* element, RigidBody* selectedBody);

    void updateWidget();
    RigidBody* getBody() {return selectedBody;}
    void setBody(RigidBody* body_);
    void setSavedBodyOfReference(const QString &str) {saved_bodyOfReference = str;}
    const QString& getSavedBodyOfReference() const {return saved_bodyOfReference;}

  protected:
    QLineEdit* body;
    Element* element;
    RigidBodyBrowser* bodyBrowser;
    RigidBody* selectedBody;
    QString saved_bodyOfReference;

  public slots:
    void setBody();

  signals:
    void bodyChanged();
};

class FileWidget : public Widget {
  Q_OBJECT

  public:
    FileWidget(const QString &description, const QString &extensions);
    QString getFileName() const {return fileName->text();}
    void setFileName(const QString &str) {fileName->setText(str);}
    QString getAbsoluteFilePath() const {return absoluteFilePath;}
    void setAbsoluteFilePath(const QString &str) {absoluteFilePath = str;}

  protected:
    QLineEdit *fileName;
    QString absoluteFilePath, description, extensions;

  protected slots:
    void selectFile();

};

class TextWidget : public Widget {

  public:
    TextWidget(bool readOnly=false);

    QString getText() const {return text->text();}
    void setText(const QString &name) {text->setText(name);}

  protected:
    QLineEdit *text;
};

class ConnectFramesWidget : public Widget {

  friend class ConnectFramesProperty;

  public:
    ConnectFramesWidget(int n, Element* element);

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

class DependenciesWidget : public Widget {
  Q_OBJECT

  friend class DependenciesProperty;

  public:
    DependenciesWidget(Element* element);

    void updateWidget(); 

    RigidBody* getBody(int i) {return refBody[i]->getBody();}
    void addBody(int i, RigidBody* body_);
    int getSize() const {return refBody.size();}
    void setNumberOfBodies(int n);

  protected:
    Element* element;
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

class SolverTolerancesWidget : public Widget {
  friend class SolverTolerancesProperty;

  public:
    SolverTolerancesWidget();

  protected:
    ExtWidget *projection, *g, *gd, *gdd, *la, *La;
};

class SolverParametersWidget : public Widget {

  friend class SolverParametersProperty;

  public:
    SolverParametersWidget();

  protected:
    ExtWidget *tolerances;
};

class PlotFeature : public Widget {
  public:
    PlotFeature(const std::string &name);

  protected:
    std::string name;
    QComboBox *status;
};

class GearDependencyWidget : public Widget {

  friend class GearDependencyProperty;

  public:
    GearDependencyWidget(Element* element);
    RigidBody* getBody() {return refBody->getBody();}
    RigidBodyOfReferenceWidget* getRigidBodyOfReferenceWidget() {return refBody;}
    void updateWidget() {refBody->updateWidget();}
  protected:
   RigidBodyOfReferenceWidget* refBody;
   ExtWidget *ratio;
};

class GearDependenciesWidget : public Widget {
  Q_OBJECT

  friend class GearDependenciesProperty;

  public:
    GearDependenciesWidget(Element* element);

    void updateWidget(); 

    RigidBody* getBody(int i) {return refBody[i]->getBody();}
    void addBody(int i, RigidBody* body_);
    int getSize() const {return refBody.size();}
    void setNumberOfBodies(int n);

  protected:
    Element* element;
    std::vector<RigidBody*> selectedBody;
    std::vector<GearDependencyWidget*> refBody;
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

#endif
