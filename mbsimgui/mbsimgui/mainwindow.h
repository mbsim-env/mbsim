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
#include <boost/filesystem/path.hpp>
#include <xercesc/util/XercesDefs.hpp>

class QAction;
class QModelIndex;

namespace OpenMBVGUI {
  class MainWindow;
}

namespace MBXMLUtils {
  class Eval;
  class NewParamLevel;
  class DOMParser;
}

namespace XERCES_CPP_NAMESPACE {
  class DOMDocument;
  class DOMElement;
}

namespace MBSimGUI {

  class Process;
  class MBSimThread;
  class ElementView;
  class EmbeddingView;
  class SolverView;
  class Element;
  class Frame;
  class Contour;
  class Group;
  class Object;
  class Link;
  class Observer;
  class Parameter;
  class ParameterList;

  class MainWindow : public QMainWindow {

    Q_OBJECT

    private:
      ElementView *elementList;
      EmbeddingView *embeddingList;
      SolverView *solverView;
      QString fileProject; 
      Process *mbsim;
      MBSimThread *mbsimThread;
      OpenMBVGUI::MainWindow *inlineOpenMBVMW;
      void initInlineOpenMBV();
      void dragEnterEvent(QDragEnterEvent *event);
      void dropEvent(QDropEvent *event);
      bool maybeSave();
      boost::filesystem::path uniqueTempDir;
      QAction *actionSaveProject, *actionSaveMBS, *actionSimulate, *actionOpenMBV, *actionH5plotserie, *actionSaveIntegrator, *actionSaveParameterList, *actionSaveDataAs, *actionSaveMBSimH5DataAs, *actionSaveOpenMBVDataAs, *actionRefresh, *actionSaveStateVectorAs, *actionSaveEigenanalysisAs;
      QTimer *autoSaveTimer;
      std::string currentID;
      enum { maxRecentFiles = 5 };
      QAction *recentProjectFileActs[maxRecentFiles];
      void setCurrentProjectFile(const QString &fileName);
      void updateRecentProjectFileActions();
      bool autoSave, autoExport, saveFinalStateVector;
      int autoSaveInterval;
      QString autoExportDir;

    public:
      MainWindow(QStringList &arg);
      ~MainWindow();
      boost::shared_ptr<MBXMLUtils::DOMParser> parser;
      boost::shared_ptr<MBXMLUtils::Eval> eval;
      void mbsimxml(int task);
      const boost::filesystem::path& getUniqueTempDir() const {return uniqueTempDir;}
      void addParameter(Parameter *parameter);
      void addFrame(Frame *frame);
      void addContour(Contour *contour);
      void addGroup(Group *group);
      void addObject(Object *object);
      void addLink(Link *link);
      void addObserver(Observer *observer);
      void highlightObject(const std::string &ID);
      const std::string& getHighlightedObject() const {return currentID;}
      void loadProject(const QString &file);
      ElementView* getElementList() { return elementList; }
      void setProjectChanged(bool changed=true);
    public slots:
      void elementListClicked();
      void parameterListClicked();
      void newProject(bool ask=true);
      void loadProject();
      bool saveProjectAs();
      bool saveProject(const QString &filename="");
      void selectIntegrator();
      void selectDOPRI5Integrator();
      void selectRADAU5Integrator();
      void selectLSODEIntegrator();
      void selectLSODARIntegrator();
      void selectTimeSteppingIntegrator();
      void selectEulerExplicitIntegrator();
      void selectRKSuiteIntegrator();
      void selectEigenanalyser();
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
      void removeParameter();
      void simulate();
      void refresh();
      void openmbv();
      void h5plotserie();
      void help();
      void about();
      void updateParameters(Element *element);
      void removeElement();
      void saveElementAs();
    protected slots:
      void selectElement(std::string);
      void changeWorkingDir();
      void openOptionsMenu();
      void selectionChanged(const QModelIndex &current, const QModelIndex &previous);
      void simulationFinished(int exitCode, QProcess::ExitStatus exitStatus);
      void openRecentProjectFile();
      void preprocessFinished(int result);
      void autoSaveProject();
    protected:
      void closeEvent ( QCloseEvent * event );

      // write parameter list to XML. The returned DOMNodes are owned by doc.
      xercesc::DOMElement* writeProject(boost::shared_ptr<xercesc::DOMDocument> &doc);
  };

}

#endif
