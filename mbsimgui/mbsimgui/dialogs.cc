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

#include <config.h>
#include "dialogs.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "rigid_body.h"
#include "signal_.h"
#include "constraint.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "mainwindow.h"
#include "element_view.h"
#include "treemodel.h"
#include "treeitem.h"
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QTreeWidget>
#include <QTableWidget>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QButtonGroup>
#include <QRadioButton>
#include <QMessageBox>
#include <QFileInfo>
#include <QSettings>
#include <qwt_plot.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_curve.h>
#include <qwt_legend.h>
#include <qwt_plot_grid.h>
#include <qwt_symbol.h>
#include <qwt_scale_engine.h>
#include <boost/math/constants/constants.hpp>
#include "hdf5serie/file.h"
#include "hdf5serie/simpledataset.h"

using namespace std;
using namespace boost::math::constants;
using namespace fmatvec;

namespace MBSimGUI {

  extern MainWindow *mw;

  QModelIndex findTreeItemData(QModelIndex root, TreeItemData *sel) {
    auto *model = static_cast<const ElementTreeModel*>(root.model());
    for(int i=0; i<model->rowCount(root); i++) {
      if(model->getItem(root.model()->index(i,0,root))->getItemData()==sel)
        return root.model()->index(i,0,root);
      QModelIndex index = findTreeItemData(root.model()->index(i,0,root), sel);
      if(index.isValid())
        return index;
    }
    return QModelIndex();
  }

  void Dialog::showEvent(QShowEvent *event) {
    QSettings settings;
    restoreGeometry(settings.value("dialog/geometry").toByteArray());
    QDialog::showEvent(event);
  }

  void Dialog::hideEvent(QHideEvent *event) {
    QSettings settings;
    settings.setValue("dialog/geometry", saveGeometry());
    QDialog::hideEvent(event);
  }

  EvalDialog::EvalDialog(const vector<vector<QString>> &var_, int type_, QWidget *parent) : Dialog(parent), var(var_), varf(var_), type(type_) {

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto *layout = new QGridLayout;
    mainlayout->addLayout(layout);

    if(type==0) {
      layout->addWidget(new QLabel("Format:"),0,0);
      format = new QComboBox;
      format->addItems(QStringList() << "e" << "E" << "f" << "g" << "G");
      format->setCurrentIndex(3);
      layout->addWidget(format,0,1);
      connect(format, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &EvalDialog::updateWidget);

      layout->addWidget(new QLabel("Precision:"),0,2);
      precision = new QSpinBox;
      precision->setValue(6);
      layout->addWidget(precision,0,3);
      connect(precision, QOverload<int>::of(&QSpinBox::valueChanged), this, &EvalDialog::updateWidget);
    }

    formatVariables();

    tab = new QTableWidget;
    tab->setRowCount(varf.size());
    tab->setColumnCount(!varf.empty()?varf[0].size():0);
    for(int i=0; i<tab->rowCount(); i++) {
      for(int j=0; j<tab->columnCount(); j++)
        tab->setItem(i,j,new QTableWidgetItem(varf[i][j]));
    }

    layout->addWidget(tab,1,0,1,5);

    auto *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Close);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &EvalDialog::reject);

    mainlayout->addWidget(buttonBox);

    layout->setColumnStretch(4, 10);

    setWindowTitle("Expression evaluation");
  }

  void EvalDialog::formatVariables() {
    if(type==0) {
      QString f = format->currentText();
      int p = precision->value();
      for(int i=0; i<var.size(); i++) {
        for(int j=0; j<var[i].size(); j++)
          varf[i][j] = QString::number(var[i][j].toDouble(),f[0].toLatin1(),p);
      }
    }
  }

  void EvalDialog::updateWidget() {
    formatVariables();
    for(int i=0; i<tab->rowCount(); i++) {
      for(int j=0; j<tab->columnCount(); j++)
        tab->item(i,j)->setText(varf[i][j]);
    }
  }

  BasicElementBrowser::BasicElementBrowser(Element* selection_, const QString &name, QWidget *parent) : Dialog(parent), selection(selection_) {
    auto* mainLayout=new QGridLayout;
    setLayout(mainLayout);
    eleList = new QTreeView;
    eleList->setModel(mw->getElementView()->model());
    eleList->setColumnWidth(0,250);
    eleList->setColumnWidth(1,200);
    eleList->hideColumn(1);
    mainLayout->addWidget(eleList,0,0);
    connect(eleList, &QTreeView::pressed, this, &BasicElementBrowser::selectionChanged);

    okButton = new QPushButton("Ok");
    if(!selection)
      okButton->setDisabled(true);
    mainLayout->addWidget(okButton,1,0);
    connect(okButton, &QPushButton::clicked, this, &BasicElementBrowser::accept);

    QPushButton *button = new QPushButton("Cancel");
    mainLayout->addWidget(button,1,1);
    connect(button, &QPushButton::clicked, this, &BasicElementBrowser::reject);

    setWindowTitle(name+" browser");
  }

  void BasicElementBrowser::showEvent(QShowEvent *event) {
    Dialog::showEvent(event);
    oldID = mw->getHighlightedObject();
    QModelIndex index1 = findTreeItemData(eleList->model()->index(0,0),selection);
    QModelIndex index2 = mw->getElementView()->selectionModel()->currentIndex().parent().parent();
    eleList->setCurrentIndex(index1.isValid()?index1:index2);
    if(selection) {
      mw->highlightObject(selection->getID());
      okButton->setDisabled(false);
    }
  }

  void BasicElementBrowser::hideEvent(QHideEvent *event) {
    Dialog::hideEvent(event);
    mw->highlightObject(oldID);
  }

  void BasicElementBrowser::selectionChanged(const QModelIndex &current) {
    auto *model = static_cast<ElementTreeModel*>(eleList->model());
    auto *element = dynamic_cast<Element*>(model->getItem(current)->getItemData());
    if(checkForElement(element)) {
      selection = element;
      mw->highlightObject(element->getID());
      okButton->setDisabled(false);
    }
    else {
      mw->highlightObject("");
      okButton->setDisabled(true);
    }
  }

  SourceCodeDialog::SourceCodeDialog(const QString &text, bool readOnly, QWidget *parent) : Dialog(parent) {
    setWindowTitle(QString("XML view"));
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    xmlEditor = new XMLEditorWidget(text);
    layout->addWidget(xmlEditor);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Ok);
    layout->addWidget(buttonBox);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &SourceCodeDialog::accept);
    xmlEditor->getEditor()->setReadOnly(readOnly);
  }

  void SourceCodeDialog::highlightLine(int n) {
    QTextCursor c = xmlEditor->getEditor()->textCursor();
    c.movePosition(QTextCursor::Down,QTextCursor::MoveAnchor,n);
    xmlEditor->getEditor()->setTextCursor(c);
    QList<QTextEdit::ExtraSelection> extraSelections;
    QTextEdit::ExtraSelection selection;
    QColor lineColor = QColor(Qt::yellow).lighter(160);
    selection.format.setBackground(lineColor);
    selection.format.setProperty(QTextFormat::FullWidthSelection, true);
    selection.cursor = c;
    selection.cursor.clearSelection();
    extraSelections.append(selection);
    xmlEditor->getEditor()->setExtraSelections(extraSelections);
  }

  StateTableDialog::StateTableDialog(QWidget *parent) : Dialog(parent) {
    setWindowTitle(QString("State table"));
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    stateTable = new QTreeWidget;
    layout->addWidget(stateTable);
    stateTable->setHeaderLabels(QStringList{"State number","Element name","State label","Label number"});
    stateTable->resizeColumnToContents(1);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Ok);
    layout->addWidget(buttonBox);
    updateWidget();
    connect(buttonBox, &QDialogButtonBox::accepted, this, &StateTableDialog::accept);
  }

  void StateTableDialog::updateWidget() {
    ifstream is(mw->getUniqueTempDir().generic_string()+"/statetable.asc");
    QVector<QString> name;
    QVector<QString> label;
    QVector<QString> number;
    string name_;
    string label_;
    int number_;
    while(true) {
      getline(is,name_);
      if(is.eof())
	break;
      stringstream sstr(name_);
      int pos = name_.find_last_of(']');
      sstr.seekg(pos+1);
      sstr >> label_ >> number_;
      name.append(QString::fromStdString(name_.substr(0,pos)+"]"));
      label.append(QString::fromStdString(label_));
      number.append(QString::number(number_+1));
    }
    is.close();
    stateTable->clear();
    for(unsigned int i=0; i<name.size(); i++) {
      auto *item = new QTreeWidgetItem;
      item->setText(0, QString::number(i+1));
      item->setText(1, name[i]);
      item->setText(2, label[i]);
      item->setText(3, number[i]);
      stateTable->addTopLevelItem(item);
    }
  }

  LoadModelDialog::LoadModelDialog(const QString &title) {
    setWindowTitle(title);
    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    mainlayout->addLayout(layout);

    modelFile = new ExtWidget("Model file", new FileWidget("", "Open model file", "MBSim model files (*.mbsmx);;XML files (*.xml);;All files (*.*)", 0, false),false,false,"");
    layout->addWidget(modelFile);

    mOpt = new QButtonGroup(this);
    QRadioButton *radio1 = new QRadioButton("Import model");
    QRadioButton *radio2 = new QRadioButton("Reference model file");
    radio1->setChecked(true);
    mOpt->addButton(radio1);
    mOpt->addButton(radio2);

    Widget *widget = new Widget;
    QHBoxLayout *hl = new QHBoxLayout;
    hl->setMargin(0);
    widget->setLayout(hl);
    hl->addWidget(radio1);
    hl->addWidget(radio2);
    e = new ExtWidget("Option",widget,false,false,"");
    layout->addWidget(e);

    parameterFile = new ExtWidget("Parameter file", new FileWidget("", "Open parameter file", "MBSim parameter files (*.mbspx);;XML files (*.xml);;All files (*.*)", 0, false),true,false,"");
    layout->addWidget(parameterFile);

    connect(static_cast<FileWidget*>(modelFile->getWidget()),&FileWidget::valueChanged,this,&LoadModelDialog::modelFileChanged);

    pOpt = new QButtonGroup(this);
    radio1 = new QRadioButton("Import parameters");
    radio2 = new QRadioButton("Reference parameter file");
    radio1->setChecked(true);
    pOpt->addButton(radio1);
    pOpt->addButton(radio2);

    widget = new Widget;
    hl = new QHBoxLayout;
    hl->setMargin(0);
    widget->setLayout(hl);
    hl->addWidget(radio1);
    hl->addWidget(radio2);
    e = new ExtWidget("Option",widget,true,false,"");
    layout->addWidget(e);

    connect(parameterFile,&ExtWidget::clicked,e,&ExtWidget::setActive);
    connect(e,&ExtWidget::clicked,parameterFile,&ExtWidget::setActive);

    layout->addStretch(1);

    auto *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &LoadModelDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &LoadModelDialog::reject);
    mainlayout->addWidget(buttonBox);
  }

  QString LoadModelDialog::getModelFileName() const {
    return static_cast<FileWidget*>(modelFile->getWidget())->getFile();
  }

  QString LoadModelDialog::getParameterFileName() const {
    return parameterFile->isActive()?static_cast<FileWidget*>(parameterFile->getWidget())->getFile():"";
  }

  bool LoadModelDialog::referenceModel() const {
    return mOpt->button(-3)->isChecked();
  }

  bool LoadModelDialog::referenceParameter() const {
    return pOpt->button(-3)->isChecked();
  }

  bool LoadModelDialog::getAbsoluteModelFilePath() const {
    return static_cast<FileWidget*>(modelFile->getWidget())->getAbsolutePath();
  }

  bool LoadModelDialog::getAbsoluteParameterFilePath() const {
    return static_cast<FileWidget*>(parameterFile->getWidget())->getAbsolutePath();
  }

  void LoadModelDialog::modelFileChanged(const QString &fileName) {
    QFileInfo fileInfo(fileName);
    QString pFileName = fileName;
    pFileName.replace(pFileName.size()-2,1,'p');
    if(QFileInfo::exists(mw->getProjectDir().absoluteFilePath(pFileName))) {
      static_cast<FileWidget*>(parameterFile->getWidget())->setFile(pFileName);
      parameterFile->setActive(true);
      e->setActive(true);
    }
    else {
      static_cast<FileWidget*>(parameterFile->getWidget())->setFile("");
      parameterFile->setActive(false);
      e->setActive(false);
    }
  }

  SaveModelDialog::SaveModelDialog(const QString &title, const QString &fileName, bool param) {
    QString pFileName = fileName;
    pFileName.replace(pFileName.size()-2,1,'p');

    setWindowTitle(title);
    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    mainlayout->addLayout(layout);

    modelFile = new ExtWidget("Model file", new FileWidget(fileName, "Save model file", "MBSim model files (*.mbsmx);;XML files (*.xml);;All files (*.*)", 1, false, false, QFileDialog::DontConfirmOverwrite),false,false,"");
    layout->addWidget(modelFile);
    if(param) {
      parameterFile = new ExtWidget("Parameter file", new FileWidget(pFileName, "Save parameter file", "MBSim parameter files (*.mbspx);;XML files (*.xml);;All files (*.*)", 1, false, false, QFileDialog::DontConfirmOverwrite),true,true,"");
      connect(static_cast<FileWidget*>(modelFile->getWidget()),&FileWidget::valueChanged,this,&SaveModelDialog::modelFileChanged);
      layout->addWidget(parameterFile);
    }
    else
      parameterFile = nullptr;
    layout->addStretch(1);

    auto *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &SaveModelDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &SaveModelDialog::reject);
    mainlayout->addWidget(buttonBox);
  }

  QString SaveModelDialog::getModelFileName() const {
    return static_cast<FileWidget*>(modelFile->getWidget())->getFile();
  }

  QString SaveModelDialog::getParameterFileName() const {
    return (parameterFile and parameterFile->isActive())?static_cast<FileWidget*>(parameterFile->getWidget())->getFile():"";
  }

  void SaveModelDialog::modelFileChanged(const QString &fileName) {
    QFileInfo fileInfo(fileName);
    QString pFileName = fileName;
    pFileName.replace(pFileName.size()-2,1,'p');
    static_cast<FileWidget*>(parameterFile->getWidget())->setFile(pFileName);
  }

  LoadParameterDialog::LoadParameterDialog() {
    setWindowTitle("Import/Reference Parameters");
    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    mainlayout->addLayout(layout);

    parameterFile = new ExtWidget("Parameter file", new FileWidget("", "Open parameter file", "MBSim parameter files (*.mbspx);;XML files (*.xml);;All files (*.*)", 0, false),false,false,"");
    layout->addWidget(parameterFile);

    pOpt = new QButtonGroup(this);
    QRadioButton *radio1 = new QRadioButton("Import parameters");
    QRadioButton *radio2 = new QRadioButton("Reference parameter file");
    radio1->setChecked(true);
    pOpt->addButton(radio1);
    pOpt->addButton(radio2);

    Widget *widget = new Widget;
    QHBoxLayout *hl = new QHBoxLayout;
    hl->setMargin(0);
    widget->setLayout(hl);
    hl->addWidget(radio1);
    hl->addWidget(radio2);
    ExtWidget *e = new ExtWidget("Option",widget,false,false,"");
    layout->addWidget(e);

    layout->addStretch(1);

    auto *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &LoadParameterDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &LoadParameterDialog::reject);
    mainlayout->addWidget(buttonBox);
  }

  QString LoadParameterDialog::getParameterFileName() const {
    return static_cast<FileWidget*>(parameterFile->getWidget())->getFile();
  }

  bool LoadParameterDialog::referenceParameter() const {
    return pOpt->button(-3)->isChecked();
  }

  bool LoadParameterDialog::getAbsoluteFilePath() const {
    return static_cast<FileWidget*>(parameterFile->getWidget())->getAbsolutePath();
  }

  SaveParameterDialog::SaveParameterDialog(const QString &fileName) {
    setWindowTitle("Export Parameters");
    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    mainlayout->addLayout(layout);

    parameterFile = new ExtWidget("Parameter file", new FileWidget(fileName, "Save parameter file", "MBSim parameter files (*.mbspx);;XML files (*.xml);;All files (*.*)", 1, false, false, QFileDialog::DontConfirmOverwrite),false,false,"");
    layout->addWidget(parameterFile);

    layout->addStretch(1);

    auto *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &SaveParameterDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &SaveParameterDialog::reject);
    mainlayout->addWidget(buttonBox);
  }

  QString SaveParameterDialog::getParameterFileName() const {
    return static_cast<FileWidget*>(parameterFile->getWidget())->getFile();
  }

  InitialOutputWidget::InitialOutputWidget() {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    text = new QTextEdit;
    layout->addWidget(text);
    loadData();
  }

  void InitialOutputWidget::loadData() {
    H5::File file(mw->getUniqueTempDir().generic_string()+"/linear_system_analysis.h5", H5::File::read);
    auto group=file.openChildObject<H5::Group>("initial output");
    auto data=group->openChildObject<H5::SimpleDataset<vector<double>>>("state (z)");
    auto z = data->read();
    data=group->openChildObject<H5::SimpleDataset<vector<double>>>("output (y)");
    auto y = data->read();
    stringstream stream;
    stream << "State (z)" << endl;
    for(size_t i=0; i<z.size(); i++)
      stream << setw(28) << z[i] << endl;
    stream << endl << "Output (y)" << endl;
    for(size_t i=0; i<y.size(); i++)
      stream << setw(28) << y[i] << endl;
    text->setPlainText(QString::fromStdString(stream.str()));
  }

  EigenanalysisWidget::EigenanalysisWidget() {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    text = new QTextEdit;
    layout->addWidget(text);
    loadData();
  }

  void EigenanalysisWidget::loadData() {
    H5::File file(mw->getUniqueTempDir().generic_string()+"/linear_system_analysis.h5", H5::File::read);
    auto group=file.openChildObject<H5::Group>("eigenanalysis");
    auto cvdata = group->openChildObject<H5::SimpleDataset<vector<complex<double>>>>("eigenvalues");
    auto w = cvdata->read();
    auto cmdata = group->openChildObject<H5::SimpleDataset<vector<vector<complex<double>>>>>("eigenvectors");
    auto V = cmdata->read();
    stringstream stream;
    stream << "Eigenvalues" << endl;
    for(size_t i=0; i<w.size(); i++)
      stream << setw(28) << w[i] << endl;
    stream << endl << "Eigenvectors" << endl;
    for(size_t i=0; i<V.size(); i++) {
      for(size_t j=0; j<V[0].size(); j++)
	stream << setw(28) << V[i][j] << " ";
      stream << endl;
    }
    text->setPlainText(QString::fromStdString(stream.str()));
  }

  ModalAnalysisWidget::ModalAnalysisWidget() {
    auto *layout = new QGridLayout;
    setLayout(layout);

    modeTable = new QTreeWidget;
    layout->addWidget(modeTable,0,0);
    modeTable->setHeaderLabels(QStringList{"Mode number","Natural frequency","Expotential decay","Natural angular frequency","Damping ratio"});

    choice = new QComboBox;
    layout->addWidget(choice,1,0);

    auto *scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);

    plot = new QwtPlot(scrollArea);
    scrollArea->setWidget(plot);
    QwtLegend *legend = new QwtLegend;
    plot->insertLegend(legend,QwtPlot::BottomLegend);
    plot->setAxisTitle(QwtPlot::yLeft,"Normalized Amplitude");
    plot->setAxisTitle(QwtPlot::yRight,"Phase (deg)");
    plot->setAxisScale(QwtPlot::yRight,-180,180,45);
    plot->setCanvasBackground(Qt::white);

    curve1 = new QwtPlotCurve;
    curve1->setTitle("Amplitude");
    curve1->setSymbol(new QwtSymbol(QwtSymbol::Diamond,QColor(Qt::red),QColor(Qt::black),QSize(10,10)));
    curve1->setRenderHint(QwtPlotItem::RenderAntialiased);
    curve1->setPen(Qt::red);
    curve1->setYAxis(QwtPlot::yLeft);
    curve1->attach(plot);
    plot->enableAxis(QwtPlot::yRight);

    curve2 = new QwtPlotCurve;
    curve2->setTitle("Phase");
    curve2->setSymbol(new QwtSymbol(QwtSymbol::Diamond,QColor(Qt::green),QColor(Qt::black),QSize(10,10)));
    curve2->setRenderHint(QwtPlotItem::RenderAntialiased);
    curve1->setLegendAttribute(QwtPlotCurve::LegendShowLine);
    curve2->setLegendAttribute(QwtPlotCurve::LegendShowLine);
    curve2->setPen(Qt::green);
    curve2->setYAxis(QwtPlot::yRight);
    curve2->attach(plot);

    QwtPlotGrid *grid = new QwtPlotGrid;
    grid->enableYMin(true);
    grid->setMajorPen(Qt::black,0,Qt::DotLine);
    grid->setMinorPen(Qt::white,0,Qt::DotLine);
    grid->attach(plot);

    layout->addWidget(scrollArea,0,1,3,1);
    layout->setColumnStretch(0,1);
    layout->setColumnStretch(1,2);

    elementTable = new QTreeWidget;
    layout->addWidget(elementTable,2,0);

    loadData();

    connect(modeTable, &QTreeWidget::currentItemChanged, this, &ModalAnalysisWidget::updateWidget);
    connect(choice, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ModalAnalysisWidget::updateWidget);
  }

  void ModalAnalysisWidget::loadData() {
    H5::File file(mw->getUniqueTempDir().generic_string()+"/linear_system_analysis.h5", H5::File::read);
    auto group=file.openChildObject<H5::Group>("modal analysis");
    auto cvdata = group->openChildObject<H5::SimpleDataset<vector<complex<double>>>>("eigenvalues");
    auto w = cvdata->read();
    auto cmdata = group->openChildObject<H5::SimpleDataset<vector<vector<complex<double>>>>>("state modes");
    auto Zh = cmdata->read();
    cmdata = group->openChildObject<H5::SimpleDataset<vector<vector<complex<double>>>>>("output modes");
    auto Yh = cmdata->read();

    string name;
    char label;
    int number;

    stateName.clear();
    stateLabel.clear();
    stateLabelNumber.clear();
    ifstream is(mw->getUniqueTempDir().generic_string()+"/statetable.asc");
     while(true) {
      getline(is,name);
      if(is.eof())
	break;
      stringstream sstr(name);
      int pos = name.find_last_of(']');
      sstr.seekg(pos+1);
      sstr >> label >> number;
      stateName.append(QString::fromStdString(name.substr(0,pos)+"]"));
      stateLabel.append(QString(label));
      stateLabelNumber.append(number);
    }
    is.close();

    outputName.clear();
    outputLabel.clear();
    outputLabelNumber.clear();
    is.open(mw->getUniqueTempDir().generic_string()+"/outputtable.asc");
    while(true) {
      getline(is,name);
      if(is.eof())
	break;
      stringstream sstr(name);
      int pos = name.find_last_of(']');
      sstr.seekg(pos+1);
      sstr >> label >> number;
      outputName.append(QString::fromStdString(name.substr(0,pos)+"]"));
      outputLabel.append(QString(label));
      outputLabelNumber.append(number);
    }
    is.close();

    num.clear();
    A.clear();
    phi.clear();
    for(int k=0; k<Zh.size(); k++)
      num[stateLabel[k]].append(k+1);
    for(int k=0; k<Yh.size(); k++)
      num[outputLabel[k]].append(k+1);
    for(QMap<QString,QVector<double>>::iterator i=num.begin(); i!=num.end(); i++) {
      A[i.key()].resize(w.size());
      phi[i.key()].resize(w.size());
      for(int j=0; j<w.size(); j++) {
	A[i.key()][j].resize(i.value().size());
	phi[i.key()][j].resize(i.value().size());
      }
    }

    modeTable->blockSignals(true);
    QString item;
    if(modeTable->currentItem()) item = modeTable->currentItem()->text(0);
    modeTable->clear();
    for(int i=0; i<w.size(); i++) {
      auto *item = new QTreeWidgetItem;
      item->setText(0, QString::number(i+1));
      item->setText(1, QString::number(w[i].imag()/2/M_PI));
      item->setText(2, QString::number(-w[i].real()));
      item->setText(3, QString::number(w[i].imag()));
      item->setText(4, QString::number(-w[i].real()/abs(w[i])));
      modeTable->addTopLevelItem(item);
      int l=0;
      for(QMap<QString,QVector<double>>::iterator j=num.begin(); j!=num.end(); j++) {
	if(j.key()=="y") {
	  for(int k=0; k<j.value().size(); k++) {
	    A[j.key()][i][k] = abs(Yh[k][i]);
	    phi[j.key()][i][k] = atan2(Yh[k][i].real(),-Yh[k][i].imag())*180/M_PI;
	  }
	}
	else {
	  for(int k=0; k<j.value().size(); k++) {
	    A[j.key()][i][k] = abs(Zh[l][i]);
	    phi[j.key()][i][k] = atan2(Zh[l][i].real(),-Zh[l][i].imag())*180/M_PI;
	    l++;
	  }
	}
      }
    }
    modeTable->resizeColumnToContents(1);
    auto itemlist = modeTable->findItems(item,Qt::MatchExactly);
    if(itemlist.size())
      modeTable->setCurrentItem(itemlist.at(0));
    else
      modeTable->setCurrentItem(modeTable->topLevelItem(0));
    modeTable->blockSignals(false);

    choice->blockSignals(true);
    auto text = choice->currentText();
    choice->clear();
    for(QMap<QString,QVector<double>>::iterator i=num.begin(); i!=num.end(); i++)
      choice->addItem((i.key()=="y"?"Output (":"State (")+i.key()+")");
    choice->setCurrentText(text);
    choice->blockSignals(false);

    updateWidget();
  }

  void ModalAnalysisWidget::updateWidget() {
    if(not modeTable->topLevelItemCount()) {
      elementTable->clear();
      curve1->setSamples(QVector<double>(),QVector<double>());
      curve2->setSamples(QVector<double>(),QVector<double>());
      return;
    }
    QString c = choice->currentText();
    c = c.mid(c.size()-2,1);
    int m = modeTable->indexOfTopLevelItem(modeTable->currentItem());
    plot->setTitle("Mode " + modeTable->currentItem()->text(0) + " of " + choice->currentText());
    curve1->setSamples(num[c],A[c][m]);
    curve2->setSamples(num[c],phi[c][m]);
    plot->setAxisScale(QwtPlot::xBottom,num[c][0],num[c][num[c].size()-1],1);
    plot->replot();
    elementTable->clear();
    if(c=="y") {
      plot->setAxisTitle(QwtPlot::xBottom,"Output number");
      elementTable->setHeaderLabels(QStringList{"Output number","Element name","Output label","Label number"});
      for(unsigned int i=0; i<outputName.size(); i++) {
	auto *item = new QTreeWidgetItem;
	item->setText(0, QString::number(i+1));
	item->setText(1, outputName[i]);
	item->setText(2, outputLabel[i]);
	item->setText(3, QString::number(outputLabelNumber[i]+1));
	elementTable->addTopLevelItem(item);
      }
    }
    else {
      plot->setAxisTitle(QwtPlot::xBottom,"State number");
      elementTable->setHeaderLabels(QStringList{"State number","Element name","State label","Label number"});
      for(unsigned int i=num[c][0]-1; i<num[c][num[c].size()-1]; i++) {
	auto *item = new QTreeWidgetItem;
	item->setText(0, QString::number(i+1));
	item->setText(1, stateName[i]);
	item->setText(2, stateLabel[i]);
	item->setText(3, QString::number(stateLabelNumber[i]+1));
	elementTable->addTopLevelItem(item);
      }
    }
    elementTable->resizeColumnToContents(1);
  }

  FrequencyResponseWidget::FrequencyResponseWidget() {
    auto *layout = new QGridLayout;
    setLayout(layout);

    inputTable = new QTreeWidget;
    layout->addWidget(inputTable,0,0);
    inputTable->setHeaderLabels(QStringList{"Input number","Element name","Input label","Label number"});

    auto *scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);
    plot = new QwtPlot(scrollArea);
    scrollArea->setWidget(plot);
    //      plot->setTitle("Frequency response");
    plot->setAxisTitle(QwtPlot::xBottom,"Excitation frequency (Hz)");
    plot->setAxisTitle(QwtPlot::yLeft,"Amplitude");
    plot->setAxisTitle(QwtPlot::yRight,"Phase (deg)");
    plot->setAxisScale(QwtPlot::yRight,-181,181,45);
    plot->setCanvasBackground(Qt::white);

    curve1 = new QwtPlotCurve;
    curve1->setTitle("Amplitude");
    curve1->setSymbol(new QwtSymbol(QwtSymbol::Diamond,QColor(Qt::red),QColor(Qt::black),QSize(10,10)));
    curve1->setRenderHint(QwtPlotItem::RenderAntialiased);
    curve1->setPen(Qt::red);
    curve1->setYAxis(QwtPlot::yLeft);
    curve1->attach(plot);
    plot->enableAxis(QwtPlot::yRight);

    curve2 = new QwtPlotCurve;
    curve2->setTitle("Phase");
    curve2->setSymbol(new QwtSymbol(QwtSymbol::Diamond,QColor(Qt::green),QColor(Qt::black),QSize(10,10)));
    curve2->setRenderHint(QwtPlotItem::RenderAntialiased);
    curve1->setLegendAttribute(QwtPlotCurve::LegendShowLine);
    curve2->setLegendAttribute(QwtPlotCurve::LegendShowLine);
    curve2->setPen(Qt::green);
    curve2->setYAxis(QwtPlot::yRight);
    curve2->attach(plot);

    QwtLegend *legend = new QwtLegend;
    plot->insertLegend(legend,QwtPlot::BottomLegend);

    QwtPlotGrid *grid = new QwtPlotGrid;
    grid->enableYMin(true);
    grid->setMajorPen(Qt::black,0,Qt::DotLine);
    grid->setMinorPen(Qt::white,0,Qt::DotLine);
    grid->attach(plot);

    layout->addWidget(scrollArea,0,1,2,3);
    layout->setColumnStretch(0,1);
    layout->setColumnStretch(3,2);
    plot->replot();

    table = new QTreeWidget;
    layout->addWidget(table,1,0);
    table->setHeaderLabels(QStringList{"Magnitude","Magnitude number","Element name","Magnitude label","Label number"});

    auto *checkbox = new QCheckBox("Log scale (x)");
    layout->addWidget(checkbox,2,1);
    connect(checkbox,&QCheckBox::toggled,this,[=]() {
	if(checkbox->isChecked())
	plot->setAxisScaleEngine(QwtPlot::xBottom, new QwtLogScaleEngine);
	else
	plot->setAxisScaleEngine(QwtPlot::xBottom, new QwtLinearScaleEngine);
	plot->replot();
	});
    checkbox = new QCheckBox("Log scale (y)");
    layout->addWidget(checkbox,2,2);
    connect(checkbox,&QCheckBox::toggled,this,[=]() {
	if(checkbox->isChecked())
	plot->setAxisScaleEngine(QwtPlot::yLeft, new QwtLogScaleEngine);
	else
	plot->setAxisScaleEngine(QwtPlot::yLeft, new QwtLinearScaleEngine);
	plot->replot();
	});

    loadData();

    connect(table, &QTreeWidget::currentItemChanged, this, &FrequencyResponseWidget::updateWidget);
  }

  void FrequencyResponseWidget::loadData() {
    H5::File file(mw->getUniqueTempDir().generic_string()+"/linear_system_analysis.h5", H5::File::read);
    auto group=file.openChildObject<H5::Group>("frequency response analysis");
    auto data=group->openChildObject<H5::SimpleDataset<vector<double>>>("excitation frequencies");
    auto f = data->read();

    auto cmdata = group->openChildObject<H5::SimpleDataset<vector<vector<complex<double>>>>>("state response");
    auto Zh = cmdata->read();
    cmdata = group->openChildObject<H5::SimpleDataset<vector<vector<complex<double>>>>>("output response");
    auto Yh = cmdata->read();

    string name_;
    char label_;
    int number_;

    stateName.clear();
    stateLabel.clear();
    stateLabelNumber.clear();
    ifstream is(mw->getUniqueTempDir().generic_string()+"/statetable.asc");
    while(true) {
      getline(is,name_);
      if(is.eof())
	break;
      stringstream sstr(name_);
      int pos = name_.find_last_of(']');
      sstr.seekg(pos+1);
      sstr >> label_ >> number_;
      stateName.append(QString::fromStdString(name_.substr(0,pos)+"]"));
      stateLabel.append(QString(label_));
      stateLabelNumber.append(number_);
    }
    is.close();

    inputName.clear();
    inputLabel.clear();
    inputLabelNumber.clear();
    is.open(mw->getUniqueTempDir().generic_string()+"/inputtable.asc");
    while(true) {
      getline(is,name_);
      if(is.eof())
	break;
      stringstream sstr(name_);
      int pos = name_.find_last_of(']');
      sstr.seekg(pos+1);
      sstr >> label_ >> number_;
      inputName.append(QString::fromStdString(name_.substr(0,pos)+"]"));
      inputLabel.append(QString(label_));
      inputLabelNumber.append(number_);
    }
    is.close();

    outputName.clear();
    outputLabel.clear();
    outputLabelNumber.clear();
    is.open(mw->getUniqueTempDir().generic_string()+"/outputtable.asc");
    while(true) {
      getline(is,name_);
      if(is.eof())
	break;
      stringstream sstr(name_);
      int pos = name_.find_last_of(']');
      sstr.seekg(pos+1);
      sstr >> label_ >> number_;
      outputName.append(QString::fromStdString(name_.substr(0,pos)+"]"));
      outputLabel.append(QString(label_));
      outputLabelNumber.append(number_);
    }
    is.close();

    int rZh = Zh.size();
    int cZh = rZh?Zh[0].size():0;
    int rYh = Yh.size();
    freq = QVector<double>(f.size());
    A = QVector<QVector<double>>(rZh+rYh,QVector<double>(cZh));
    phi = QVector<QVector<double>>(rZh+rYh,QVector<double>(cZh));
    for(int i=0; i<f.size(); i++) {
      freq[i] = f[i];
      for(int j=0; j<rZh; j++) {
	A[j][i] = abs(Zh[j][i]);
	phi[j][i] = atan2(Zh[j][i].real(),-Zh[j][i].imag())*180/M_PI;
      }
      for(int j=0; j<rYh; j++) {
	A[rZh+j][i] = abs(Yh[j][i]);
	phi[rZh+j][i] = atan2(Yh[j][i].real(),-Yh[j][i].imag())*180/M_PI;
      }
    }

    inputTable->blockSignals(true);
    QString item;
    if(inputTable->currentItem()) item = inputTable->currentItem()->text(0);
    inputTable->clear();
    for(unsigned int i=0; i<inputName.size(); i++) {
      auto *item = new QTreeWidgetItem;
      item->setText(0, QString::number(i+1));
      item->setText(1, inputName[i]);
      item->setText(2, inputLabel[i]);
      item->setText(3, QString::number(inputLabelNumber[i]+1));
      inputTable->addTopLevelItem(item);
    }
    inputTable->resizeColumnToContents(1);
    auto itemlist = inputTable->findItems(item,Qt::MatchExactly);
    if(itemlist.size())
      inputTable->setCurrentItem(itemlist.at(0));
    else
      inputTable->setCurrentItem(inputTable->topLevelItem(0));
    inputTable->blockSignals(false);

    if(rZh and cZh) {
      table->blockSignals(true);
      QString item1, item2;
      if(table->currentItem()) {
	item1 = table->currentItem()->text(0);
	item2 = table->currentItem()->text(1);
      }
      table->clear();
      for(unsigned int i=0; i<stateName.size(); i++) {
	auto *item = new QTreeWidgetItem;
	item->setText(0, "State");
	item->setText(1, QString::number(i+1));
	item->setText(2, stateName[i]);
	item->setText(3, stateLabel[i]);
	item->setText(4, QString::number(stateLabelNumber[i]+1));
	table->addTopLevelItem(item);
      }
      for(unsigned int i=0; i<outputName.size(); i++) {
	auto *item = new QTreeWidgetItem;
	item->setText(0, "Output");
	item->setText(1, QString::number(i+1));
	item->setText(2, outputName[i]);
	item->setText(3, outputLabel[i]);
	item->setText(4, QString::number(outputLabelNumber[i]+1));
	table->addTopLevelItem(item);
      }
      table->resizeColumnToContents(2);
      auto itemlist = table->findItems(item2,Qt::MatchExactly,1);
      bool found = false;
      for(int i=0; i<itemlist.size(); i++) {
	if(itemlist.at(i)->text(0)==item1 and itemlist.at(i)->text(1)==item2) {
	  table->setCurrentItem(itemlist.at(i));
	  found = true;
	  break;
	}
      }
      if(not found)
	table->setCurrentItem(table->topLevelItem(0));
      table->blockSignals(false);

      updateWidget();
    }
  }

  void FrequencyResponseWidget::updateWidget() {
    plot->setTitle("Response of " + table->currentItem()->text(0) + " " + table->currentItem()->text(1));
    curve1->setSamples(freq,A[table->indexOfTopLevelItem(table->currentItem())]);
    curve2->setSamples(freq,phi[table->indexOfTopLevelItem(table->currentItem())]);
    plot->replot();
  }

  LinearSystemAnalysisDialog::LinearSystemAnalysisDialog(QWidget *parent) : Dialog(parent) {
    setWindowTitle("Linear system analysis");
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    auto *tabWidget = new QTabWidget(this);
    layout->addWidget(tabWidget);

    mawidget = new ModalAnalysisWidget;
    tabWidget->addTab(mawidget,"Modal analysis");

    frwidget = new FrequencyResponseWidget;
    tabWidget->addTab(frwidget,"Frequency response analysis");

    iowidget = new InitialOutputWidget;
    tabWidget->addTab(iowidget,"Initial output");

    eawidget = new EigenanalysisWidget;
    tabWidget->addTab(eawidget,"Eigenanalysis");

    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    layout->addWidget(buttonBox);
    buttonBox->addButton(QDialogButtonBox::Ok);

    connect(buttonBox, &QDialogButtonBox::accepted, this, &LinearSystemAnalysisDialog::accept);
  }

  void LinearSystemAnalysisDialog::updateWidget() {
    mawidget->loadData();
    frwidget->loadData();
    iowidget->loadData();
    eawidget->loadData();
  }

  CreateFMUDialog::CreateFMUDialog(const QString &fileName) {
    setWindowTitle("Create FMU file");
    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    mainlayout->addLayout(layout);

    file = new ExtWidget("FMU file", new FileWidget(fileName, "FMU file", "MBSim FMU files (*.fmu);;All files (*.*)", 1, false, false, QFileDialog::DontConfirmOverwrite),false,false,"");
    layout->addWidget(file);
    layout->addStretch(1);

    opt = new QButtonGroup(this);
    QRadioButton *radio1 = new QRadioButton("Model exchange");
    QRadioButton *radio2 = new QRadioButton("Co-simulation");
    radio1->setChecked(true);
    opt->addButton(radio1);
    opt->addButton(radio2);
    checkbox = new QCheckBox("Compression");
    checkbox->setChecked(false);

    Widget *widget = new Widget;
    QHBoxLayout *hl = new QHBoxLayout;
    hl->setMargin(0);
    widget->setLayout(hl);
    hl->addWidget(radio1);
    hl->addWidget(radio2);
    hl->addWidget(checkbox);
    ExtWidget *e = new ExtWidget("Options",widget,false,false,"");
    layout->addWidget(e);

    layout->addStretch(1);

    auto *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &SaveModelDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &SaveModelDialog::reject);
    mainlayout->addWidget(buttonBox);
  }

  QString CreateFMUDialog::getFileName() const {
    return static_cast<FileWidget*>(file->getWidget())->getFile();
  }

  bool CreateFMUDialog::cosim() const {
    return opt->button(-3)->isChecked();
  }

  bool CreateFMUDialog::nocompress() const {
    return not checkbox->isChecked();
  }

}
