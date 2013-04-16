/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2013 Martin Förg

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
#include "delegate.h"
#include "treemodel.h"
#include "treeitem.h"
#include "element.h"
#include "parameter.h"
#include "property_dialog.h"
#include "mainwindow.h"

using namespace std;

extern MainWindow *mw;

//ElementDelegate::ElementDelegate(QObject *parent) : QItemDelegate(parent) {
//  QItemEditorFactory *factory = new QItemEditorFactory;
//
//  QItemEditorCreatorBase *elementEditor =
//    new QStandardItemEditorCreator<ElementPropertyDialog>();
//
//  factory->registerEditor(QVariant::String, elementEditor);
//
//  setItemEditorFactory(factory);
//}

void ElementDelegate::commitDataAndcloseEditor(QWidget *editor) {
  emit commitData(editor);
  emit closeEditor(editor);
}

void ElementDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  QAbstractItemDelegate::updateEditorGeometry(editor,option,index);
}

QWidget *ElementDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  const ElementTreeModel *model = static_cast<const ElementTreeModel*>(index.model());
  ElementPropertyDialog *editor = static_cast<Element*>(model->getItem(index)->getItemData())->createPropertyDialog();
  editor->setModal(true);
  connect(editor,SIGNAL(apply(QWidget*)),this,SIGNAL(commitData(QWidget*)));
  connect(editor,SIGNAL(cancel(QWidget*)),this,SIGNAL(closeEditor(QWidget*)));
  connect(editor,SIGNAL(ok(QWidget*)),this,SLOT(commitDataAndcloseEditor(QWidget*)));
  return editor;
}

void ElementDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
  const ElementTreeModel *model = static_cast<const ElementTreeModel*>(index.model());
  ElementPropertyDialog *dialog = static_cast<ElementPropertyDialog*>(editor);
  dialog->toWidget(static_cast<Element*>(model->getItem(index)->getItemData()));
}

void ElementDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {
  ElementPropertyDialog *dialog = static_cast<ElementPropertyDialog*>(editor);
  dialog->fromWidget(static_cast<Element*>(static_cast<const ElementTreeModel*>(model)->getItem(index)->getItemData()));
  static_cast<ElementTreeModel*>(model)->updateView(index);
  mw->mbsimxml(1);
}

void ParameterDelegate::commitDataAndcloseEditor(QWidget *editor) {
  emit commitData(editor);
  emit closeEditor(editor);
}

QWidget *ParameterDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  const ParameterListModel *model = static_cast<const ParameterListModel*>(index.model());
  ParameterPropertyDialog *editor = static_cast<Parameter*>(model->getItem(index)->getItemData())->createPropertyDialog();
  editor->setModal(true);
  connect(editor,SIGNAL(apply(QWidget*)),this,SIGNAL(commitData(QWidget*)));
  connect(editor,SIGNAL(cancel(QWidget*)),this,SIGNAL(closeEditor(QWidget*)));
  connect(editor,SIGNAL(ok(QWidget*)),this,SLOT(commitDataAndcloseEditor(QWidget*)));
  return editor;
}

void ParameterDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
  const ParameterListModel *model = static_cast<const ParameterListModel*>(index.model());
  ParameterPropertyDialog *dialog = static_cast<ParameterPropertyDialog*>(editor);
  dialog->toWidget(static_cast<Parameter*>(model->getItem(index)->getItemData()));
}

void ParameterDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {
  ParameterPropertyDialog *dialog = static_cast<ParameterPropertyDialog*>(editor);
  dialog->fromWidget(static_cast<Parameter*>(static_cast<const ParameterListModel*>(model)->getItem(index)->getItemData()));
  static_cast<ParameterListModel*>(model)->updateView(index);
  mw->updateOctaveParameters();
  mw->mbsimxml(1);
}

void ParameterDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  QAbstractItemDelegate::updateEditorGeometry(editor,option,index);
}
