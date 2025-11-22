/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012-2014 Martin Förg

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
#include "options.h"
#include "qbuttongroup.h"
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QLabel>
#include <QCheckBox>
#include <QSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QFileDialog>
#include <QTextEdit>
#include <QGroupBox>
#include <QComboBox>
#include <evaluator/evaluator.h>

namespace MBSimGUI {

  OptionsDialog::OptionsDialog(QWidget *widget) : QDialog(widget) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    auto *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Ok);
    buttonBox->addButton(QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &OptionsDialog::reject);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &OptionsDialog::accept);

    auto *sublayout = new QHBoxLayout;
    layout->addLayout(sublayout);
    autoSave = new QCheckBox("Auto save project every");
    sublayout->addWidget(autoSave);
    autoSaveInterval = new QSpinBox;
    autoSaveInterval->setMinimum(1);
    connect(autoSave,&QCheckBox::stateChanged,this,&OptionsDialog::autoSaveChanged);
    sublayout->addWidget(autoSaveInterval);
    QLabel *label = new QLabel("min");
    sublayout->addWidget(label);

    sublayout = new QHBoxLayout;
    layout->addLayout(sublayout);
    autoExport = new QCheckBox("Auto export simulation data");
    connect(autoExport,&QCheckBox::stateChanged,this,&OptionsDialog::autoExportChanged);
    sublayout->addWidget(autoExport);
    autoExportDir = new QLineEdit("Export directory");
    sublayout->addWidget(autoExportDir);
    button = new QPushButton("Browse");
    connect(button,&QPushButton::clicked,this,&OptionsDialog::openFileBrowser);
    sublayout->addWidget(button);

    saveStateVector = new QCheckBox("Save final state vector");
    layout->addWidget(saveStateVector);

    sublayout = new QHBoxLayout;
    layout->addLayout(sublayout);
    label = new QLabel("Maximum number of undos");
    sublayout->addWidget(label);
    maxUndo = new QSpinBox;
    maxUndo->setMinimum(1);
    sublayout->addWidget(maxUndo);

    showFilters = new QCheckBox("Show list filters");
    layout->addWidget(showFilters);

    showHiddenItems = new QCheckBox("Show hidden elements/parameters");
    showHiddenItems->setToolTip("For several elements/parameters a 'hidden' flag can be set which prevents these items "
                                   "from appearing in the model/parameter tree. Settings this option will show even hidden items.");
    layout->addWidget(showHiddenItems);

    showEmptyParameters = new QCheckBox("Show parameter groups which are empty");
    showEmptyParameters->setToolTip("If disabled a parameter group which does not contain any parameter or sub group is not displayed.\n"
                                    "- positive effect: many empty groups will just not be shown at all\n"
                                    "- negative effect: a parameter can only be added if already another exists");
    layout->addWidget(showEmptyParameters);

    auto parameterViewGroup = new QGroupBox("'Parameter Tree' shows the following parameters", this);
    layout->addWidget(parameterViewGroup);
    auto parameterViewLO = new QHBoxLayout(parameterViewGroup);
    parameterViewGroup->setLayout(parameterViewLO);
    parameterViewOnlyForCurrentElement = new QRadioButton("the current selected element");
    parameterViewOnlyForCurrentElement->setToolTip("Only the parameters which influence the currently selected element are shown");
    parameterViewAll = new QRadioButton("all");
    parameterViewAll->setToolTip("All parameters of the model are shown");
    parameterViewLO->addWidget(parameterViewOnlyForCurrentElement);
    parameterViewLO->addWidget(parameterViewAll);

    statusUpdate = new QCheckBox("Always update element status");
    layout->addWidget(statusUpdate);

    auto onErrorLinks = new QGroupBox("When a error link in 'Echo Area' is clicked open", this);
    layout->addWidget(onErrorLinks);
    auto onErrorLinksLO = new QHBoxLayout(onErrorLinks);
    onErrorLinks->setLayout(onErrorLinksLO);
    onErrorLinksOpenPropertyDialog = new QRadioButton("the property dialog");
    onErrorLinksOpenPropertyDialog->setToolTip("The property dialog of the element is shown or, if its not found, the XML source");
    onErrorLinksOpenXMLSource = new QRadioButton("the XML source");
    onErrorLinksOpenPropertyDialog->setToolTip("The XML source view is shown");
    onErrorLinksLO->addWidget(onErrorLinksOpenPropertyDialog);
    onErrorLinksLO->addWidget(onErrorLinksOpenXMLSource);

    layout->addWidget(new QLabel("MBSimGUI plugin search dirs (one directory per line; libmbsimgui-plugin-*.[so|dll]):"));
    plugins = new QTextEdit;
    layout->addWidget(plugins);

    sublayout = new QHBoxLayout;
    layout->addLayout(sublayout);
    label = new QLabel("Default evaluator");
    sublayout->addWidget(label);
    defaultEvaluator = new QComboBox;
    for(auto &x : Evaluator::evaluators)
      defaultEvaluator->addItem(x.c_str());
    sublayout->addWidget(defaultEvaluator);

    auto mbsimxmlGroup = new QGroupBox("mbsimxml options");
    layout->addWidget(mbsimxmlGroup);
    auto mbsimxmlGroupLayout = new QVBoxLayout;
    mbsimxmlGroup->setLayout(mbsimxmlGroupLayout);
    mbsimxmlGroupLayout->addWidget(new QLabel("MBSim module search path (one directory per line):"));
    modulePath = new QTextEdit;
    modulePath->setLineWrapMode(QTextEdit::NoWrap);
    mbsimxmlGroupLayout->addWidget(modulePath);

    layout->addWidget(buttonBox);
    setWindowTitle("Options");
  }

  void OptionsDialog::openFileBrowser() {
    QString dir = QFileDialog::getExistingDirectory (this, "Select directory", ".");
    if(dir != "")
      setAutoExportDir(dir);
  }
  
  void OptionsDialog::autoSaveChanged(int state) {
    autoSaveInterval->setEnabled(state);
  }

  void OptionsDialog::autoExportChanged(int state) {
    autoExportDir->setEnabled(state);
    button->setEnabled(state);
  }

  bool OptionsDialog::getAutoSave() const {
    return (autoSave->checkState()==Qt::Checked);
  }

  void OptionsDialog::setAutoSave(bool flag) {
    autoSave->setCheckState(flag?Qt::Checked:Qt::Unchecked);
    autoSaveChanged(flag);
  }

  int OptionsDialog::getAutoSaveInterval() const {
    return autoSaveInterval->value();
  }

  void OptionsDialog::setAutoSaveInterval(int min) {
    autoSaveInterval->setValue(min);
  }

  bool OptionsDialog::getAutoExport() const {
    return (autoExport->checkState()==Qt::Checked);
  }

  void OptionsDialog::setAutoExport(bool flag) {
    autoExport->setCheckState(flag?Qt::Checked:Qt::Unchecked);
    autoExportChanged(flag);
  }

  QString OptionsDialog::getAutoExportDir() const {
    return autoExportDir->text();
  }

  void OptionsDialog::setAutoExportDir(const QString& dir) {
    autoExportDir->setText(dir);
  }

  bool OptionsDialog::getSaveStateVector() const {
    return (saveStateVector->checkState()==Qt::Checked);
  }

  void OptionsDialog::setSaveStateVector(bool flag) {
    saveStateVector->setCheckState(flag?Qt::Checked:Qt::Unchecked);
  }

  int OptionsDialog::getMaxUndo() const {
    return maxUndo->value();
  }

  void OptionsDialog::setMaxUndo(int num) {
    maxUndo->setValue(num);
  }

  bool OptionsDialog::getShowFilters() const {
    return (showFilters->checkState()==Qt::Checked);
  }

  void OptionsDialog::setShowFilters(bool flag) {
    showFilters->setCheckState(flag?Qt::Checked:Qt::Unchecked);
  }

  bool OptionsDialog::getShowHiddenItems() const {
    return (showHiddenItems->checkState()==Qt::Checked);
  }

  void OptionsDialog::setShowHiddenItems(bool flag) {
    showHiddenItems->setCheckState(flag?Qt::Checked:Qt::Unchecked);
  }

  bool OptionsDialog::getShowEmptyParameters() const {
    return (showEmptyParameters->checkState()==Qt::Checked);
  }

  void OptionsDialog::setShowEmptyParameters(bool flag) {
    showEmptyParameters->setCheckState(flag?Qt::Checked:Qt::Unchecked);
  }

  OptionsDialog::ParameterView OptionsDialog::getParameterView() const {
    if(parameterViewOnlyForCurrentElement->isChecked()) return ParameterView::onlyForCurrentElement;
    if(parameterViewAll                  ->isChecked()) return ParameterView::all;
    throw std::runtime_error("Internal error: 'ParameterView' is not handled.");
  }

  void OptionsDialog::setParameterView(ParameterView flag) {
    if(flag == ParameterView::onlyForCurrentElement) parameterViewOnlyForCurrentElement->setChecked(true);
    if(flag == ParameterView::all                  ) parameterViewAll                  ->setChecked(true);
  }

  bool OptionsDialog::getOpenPropertyDialogOnErrorLinks() const {
    if(onErrorLinksOpenPropertyDialog->isChecked()) return true;
    if(onErrorLinksOpenXMLSource     ->isChecked()) return false;
    throw std::runtime_error("Internal error: 'OpenPropertyDialogOnErrorLinks' is not handled.");
  }

  void OptionsDialog::setOpenPropertyDialogOnErrorLinks(bool propertyDialog) {
    if( propertyDialog) onErrorLinksOpenPropertyDialog->setChecked(true);
    if(!propertyDialog) onErrorLinksOpenXMLSource     ->setChecked(true);
  }

  bool OptionsDialog::getStatusUpdate() const {
    return (statusUpdate->checkState()==Qt::Checked);
  }

  void OptionsDialog::setStatusUpdate(bool flag) {
    statusUpdate->setCheckState(flag?Qt::Checked:Qt::Unchecked);
  }


  QString OptionsDialog::getPlugins() const {
    return plugins->toPlainText();
  }

  void OptionsDialog::setPlugins(const QString &path) {
    plugins->setPlainText(path);
  }

  int OptionsDialog::getDefaultEvaluator() const {
    return defaultEvaluator->currentIndex();
  }

  void OptionsDialog::setDefaultEvaluator(int index) {
    defaultEvaluator->setCurrentIndex(index);
  }

  QString OptionsDialog::getModulePath() const {
    return modulePath->toPlainText();
  }

  void OptionsDialog::setModulePath(const QString &path) {
    modulePath->setPlainText(path);
  }
}
