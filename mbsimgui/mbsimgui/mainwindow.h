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

#ifndef __MAINWINDOW_H_
#define __MAINWINDOW_H_

#include <QMainWindow>
#include <QProcess>
#include <QTimer>
#include <QTime>
#include <QDir>
#include <boost/filesystem/path.hpp>
#include <xercesc/util/XercesDefs.hpp>
#include <deque>
#include <sstream>
#include <openmbv/mainwindow.h>

class QAction;
class QModelIndex;

namespace OpenMBVGUI {
  class MainWindow;
  class AbstractViewFilter;
}

namespace MBXMLUtils {
  class Eval;
  class NewParamLevel;
  class DOMParser;
}

namespace XERCES_CPP_NAMESPACE {
  class DOMImplementation;
  class DOMLSParser;
  class DOMLSSerializer;
  class DOMDocument;
  class DOMElement;
}

namespace MBSimGUI {

  class MBSimThread;
  class ElementView;
  class EmbeddingView;
  class SolverView;
  class ProjectView;
  class EchoView;
  class Element;
  class Frame;
  class Contour;
  class Group;
  class Object;
  class Link;
  class Constraint;
  class Observer;
  class Parameter;
  class EmbedItemData;
  class Project;
  class EchoStream;

  class MainWindow : public QMainWindow {

    Q_OBJECT

    private:
      Project *project;
      ElementView *elementView;
      EmbeddingView *embeddingView;
      SolverView *solverView;
      ProjectView *projectView;
      EchoView *echoView;
      std::shared_ptr<bool> debugStreamFlag;
      QString projectFile;
      QProcess process;
      OpenMBVGUI::MainWindow *inlineOpenMBVMW;
      boost::filesystem::path uniqueTempDir;
      QAction *actionSaveProject, *actionSimulate, *actionOpenMBV, *actionH5plotserie, *actionEigenanalysis, *actionFrequencyResponse, *actionSaveDataAs, *actionSaveMBSimH5DataAs, *actionSaveOpenMBVDataAs, *actionRefresh, *actionDebug, *actionSaveStateVectorAs, *actionSaveEigenanalysisAs, *actionUndo, *actionRedo;
      OpenMBVGUI::AbstractViewFilter *elementViewFilter, *embeddingViewFilter;
      QTimer autoSaveTimer;
      QTime statusTime;
      QString currentID;
      enum { maxRecentFiles = 5 };
      QAction *recentProjectFileActs[maxRecentFiles];
      bool allowUndo;
      int maxUndo;
      bool autoRefresh;
      xercesc::DOMDocument *doc;
      std::deque<xercesc::DOMDocument*> undos, redos;
      std::pair<Element*,bool> elementBuffer;
      std::pair<Parameter*,bool> parameterBuffer;
      void initInlineOpenMBV();
      void dragEnterEvent(QDragEnterEvent *event) override;
      void dropEvent(QDropEvent *event) override;
      void closeEvent(QCloseEvent *event) override;
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      bool maybeSave();
      void setCurrentProjectFile(const QString &fileName);
      void updateRecentProjectFileActions();
      void move(bool up);
      void moveParameter(bool up);
      void moveFrame(bool up);
      void moveContour(bool up);
      void moveGroup(bool up);
      void moveObject(bool up);
      void moveLink(bool up);
      void moveConstraint(bool up);
      void moveObserver(bool up);

    public:
      MainWindow(QStringList &arg);
      ~MainWindow() override;
      std::shared_ptr<MBXMLUtils::DOMParser> mbxmlparser;
      std::shared_ptr<MBXMLUtils::Eval> eval;
      xercesc::DOMImplementation *impl;
      xercesc::DOMLSParser *parser;
      xercesc::DOMLSSerializer *serializer;
      xercesc::DOMLSSerializer *basicSerializer;
      void mbsimxml(int task);
      const boost::filesystem::path& getUniqueTempDir() const {return uniqueTempDir;}
      void addParameter(Parameter *parameter, EmbedItemData *parent);
      void addFrame(Frame *frame, Element *parent);
      void addContour(Contour *contour, Element *parent);
      void addGroup(Group *group, Element *parent);
      void addObject(Object *object, Element *parent);
      void addLink(Link *link, Element *parent);
      void addConstraint(Constraint *constraint, Element *parent);
      void addObserver(Observer *observer, Element *parent);
      void loadParameter(EmbedItemData *parent, Parameter *param=nullptr, bool embed=false);
      void removeParameter(EmbedItemData *parent);
      void loadFrame(Element *parent, Element *element=nullptr, bool embed=false);
      void loadContour(Element *parent, Element *element=nullptr, bool embed=false);
      void loadGroup(Element *parent, Element *element=nullptr, bool embed=false);
      void loadObject(Element *parent, Element *element=nullptr, bool embed=false);
      void loadLink(Element *parent, Element *element=nullptr, bool embed=false);
      void loadConstraint(Element *parent, Element *element=nullptr, bool embed=false);
      void loadObserver(Element *parent, Element *element=nullptr, bool embed=false);
      void highlightObject(const QString &ID);
      const QString& getHighlightedObject() const {return currentID;}
      void loadProject(const QString &file);
      ElementView* getElementView() { return elementView; }
      EmbeddingView* getEmbeddingView() { return embeddingView; }
      SolverView* getSolverView() { return solverView; }
      void setProjectChanged(bool changed=true);
      void selectSolver(int i);
      void setAllowUndo(bool allowUndo);
      const std::pair<Element*,bool>& getElementBuffer() const { return elementBuffer; }
      const std::pair<Parameter*,bool>& getParameterBuffer() const { return parameterBuffer; }
      Project* getProject() { return project; }
      QTime& getStatusTime() { return statusTime; }
      QString getProjectFilePath() const;
      QString getProjectPath() const { return QFileInfo(getProjectFilePath()).canonicalPath(); }
      QDir getProjectDir() const { return QFileInfo(getProjectFilePath()).dir(); }
      bool getAutoRefresh() const { return autoRefresh; }

    public slots:
      void elementViewClicked();
      void embeddingViewClicked();
      void solverViewClicked();
      void projectViewClicked();
      void newProject();
      void loadProject();
      bool saveProjectAs();
      bool saveProject(const QString &fileName="", bool processDocument=true, bool modifyStatus=true);
      void saveDataAs();
      void saveMBSimH5DataAs();
      void saveMBSimH5Data(const QString &file);
      void saveOpenMBVDataAs();
      void saveOpenMBVXMLData(const QString &file);
      void saveOpenMBVH5Data(const QString &file);
      void saveStateVectorAs();
      void saveStateVector(const QString &file);
      void saveEigenanalysisAs();
      void saveEigenanalysis(const QString &file);
      void simulate();
      void refresh();
      void openmbv();
      void h5plotserie();
      void eigenanalysis();
      void debug();
      void frequencyResponse();
      void help();
      void xmlHelp(const QString &url="");
      void about();
      void updateParameters(EmbedItemData *item, bool exceptLatestParameter=false);
      void rebuildTree();
      void edit();
      void undo();
      void redo();
      void removeElement();
      void removeParameter();
      void remove();
      void copy(bool cut=false);
      void cut() { copy(true); }
      void paste();
      void moveUp() { move(true); }
      void moveDown() { move(false); }
      void copyElement(bool cut=false);
      void cutElement() { copyElement(true); }
      void copyParameter(bool cut=false);
      void cutParameter() { copyParameter(true); }
      void moveUpParameter() { moveParameter(true); }
      void moveDownParameter() { moveParameter(false); }
      void moveUpFrame() { moveFrame(true); }
      void moveDownFrame() { moveFrame(false); }
      void moveUpContour() { moveContour(true); }
      void moveDownContour() { moveContour(false); }
      void moveUpGroup() { moveGroup(true); }
      void moveDownGroup() { moveGroup(false); }
      void moveUpObject() { moveObject(true); }
      void moveDownObject() { moveObject(false); }
      void moveUpLink() { moveLink(true); }
      void moveDownLink() { moveLink(false); }
      void moveUpConstraint() { moveConstraint(true); }
      void moveDownConstraint() { moveConstraint(false); }
      void moveUpObserver() { moveObserver(true); }
      void moveDownObserver() { moveObserver(false); }
      void enableElement(bool enabled);
      void saveElementAs();
      void saveSolverAs();
      void saveEmbeddingAs();
      void loadSolver();
      void viewElementSource();
      void viewEmbeddingSource();
      void viewSolverSource();
      void viewParameterSource();

    private slots:
      void selectElement(const std::string& ID);
      void openOptionsMenu(bool justSetOptions=false);
      void selectionChanged(const QModelIndex &current);
      void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
      void openRecentProjectFile();
      void autoSaveProject();
      void applySettings();
      void settingsFinished(int result);
      void interrupt();
      void kill();
      void updateEchoView();
      void updateStatus();
  };

}

#endif
