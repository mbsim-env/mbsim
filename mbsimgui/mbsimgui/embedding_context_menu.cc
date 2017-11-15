/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2017 Martin Förg

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
#include "embedding_context_menu.h"
#include "mainwindow.h"
#include "parameter.h"
#include "embeditemdata.h"
#include "embedding_view.h"
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QCheckBox>

using namespace xercesc;
using namespace MBXMLUtils;

namespace MBSimGUI {

  extern MainWindow *mw;

  class SaveDialog : public QDialog {
    public:
      SaveDialog(QWidget *parent=0);
      bool includeObject() const { return object->checkState()==Qt::Checked; }
      bool includeParameter() const { return parameter->checkState()==Qt::Checked; }
      void setParameterEnabled(bool enable) { parameter->setEnabled(enable); }
    private:
      QCheckBox *object, *parameter;
  };

  SaveDialog::SaveDialog(QWidget *parent) : QDialog(parent) {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);

    object = new QCheckBox("Embedded object");
    layout->addWidget(object);
    parameter = new QCheckBox("Embedded parameters");
    layout->addWidget(parameter);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Ok);
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    layout->addWidget(buttonBox);
  }

  EmbeddingContextMenu::EmbeddingContextMenu(EmbedItemData *item_, const QString &title, QWidget *parent) : QMenu(title,parent), item(item_) {
    QAction *action = new QAction("Edit", this);
    connect(action,SIGNAL(triggered()),mw->getEmbeddingView(),SLOT(openEditor()));
    addAction(action);
    addSeparator();
    action = new QAction("Save as", this);
    connect(action,SIGNAL(triggered()),this,SLOT(saveAs()));
    addAction(action);
    addSeparator();
    action = new QAction("Paste", this);
    action->setEnabled(mw->getParameterBuffer().first);
    connect(action,SIGNAL(triggered()),this,SLOT(paste()));
    addAction(action);
    action = new QAction("Load", this);
    connect(action,SIGNAL(triggered()),this,SLOT(load()));
    addAction(action);
    addSeparator();
    action = new QAction("Add scalar parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addScalarParameter()));
    addAction(action);
    action = new QAction("Add vector parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addVectorParameter()));
    addAction(action);
    action = new QAction("Add matrix parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addMatrixParameter()));
    addAction(action);
    action = new QAction("Add string parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addStringParameter()));
    addAction(action);
    action = new QAction("Add import parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addImportParameter()));
    addAction(action);
  }

  void EmbeddingContextMenu::saveAs() {
    SaveDialog saveDialog;
    saveDialog.setParameterEnabled(X()%item->getXMLElement()->getParentNode()->getNodeName()=="Embed" and X()%static_cast<DOMElement*>(item->getXMLElement()->getParentNode())->getFirstElementChild()->getNodeName()=="Parameter");
    saveDialog.exec();
    if(saveDialog.includeObject() or saveDialog.includeParameter())
      mw->saveEmbeddingAs(saveDialog.includeObject(),saveDialog.includeParameter());
  }

  void EmbeddingContextMenu::load() {
    mw->loadParameter(item);
  }

  void EmbeddingContextMenu::paste() {
    mw->loadParameter(item, mw->getParameterBuffer().first);
  }

  void EmbeddingContextMenu::addScalarParameter() {
    mw->addParameter(new ScalarParameter, item);
  }

  void EmbeddingContextMenu::addVectorParameter() {
    mw->addParameter(new VectorParameter, item);
  }

  void EmbeddingContextMenu::addMatrixParameter() {
    mw->addParameter(new MatrixParameter, item);
  }

  void EmbeddingContextMenu::addStringParameter() {
    mw->addParameter(new StringParameter, item);
  }

  void EmbeddingContextMenu::addImportParameter() {
    mw->addParameter(new ImportParameter, item);
  }

}
