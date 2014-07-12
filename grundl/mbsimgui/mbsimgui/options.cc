/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012-2014 Martin Förg

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

#include <config.h>
#include "options.h"
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QLabel>
#include <QCheckBox>
#include <QSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QFileDialog>

namespace MBSimGUI {

  OptionsDialog::OptionsDialog(QWidget *widget) : QDialog(widget) {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Ok);
    buttonBox->addButton(QDialogButtonBox::Cancel);
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));

    QHBoxLayout *sublayout = new QHBoxLayout;
    layout->addLayout(sublayout);
    autoSave = new QCheckBox("Auto save project every");
    sublayout->addWidget(autoSave);
    autoSaveInterval = new QSpinBox;
    autoSaveInterval->setMinimum(1);
    connect(autoSave,SIGNAL(stateChanged(int)),this,SLOT(autoSaveChanged(int)));
    sublayout->addWidget(autoSaveInterval);
    QLabel *label = new QLabel("min");
    sublayout->addWidget(label);

    sublayout = new QHBoxLayout;
    layout->addLayout(sublayout);
    autoExport = new QCheckBox("Auto export simulation data");
    connect(autoExport,SIGNAL(stateChanged(int)),this,SLOT(autoExportChanged(int)));
    sublayout->addWidget(autoExport);
    //autoExportDir = new FileWidget("Export directory", "", 3);
    autoExportDir = new QLineEdit("Export directory");
    sublayout->addWidget(autoExportDir);
    button = new QPushButton("Browse");
    connect(button,SIGNAL(clicked(bool)),this,SLOT(openFileBrowser()));
    sublayout->addWidget(button);

    saveStateVector = new QCheckBox("Save final state vector");
    layout->addWidget(saveStateVector);

    layout->addWidget(buttonBox);
    setWindowTitle("GUI options");
  }

  void OptionsDialog::openFileBrowser() {
    QString dir = QFileDialog::getExistingDirectory (0, "Select directory", ".");
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

}
