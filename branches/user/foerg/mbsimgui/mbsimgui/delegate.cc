#include "delegate.h"
#include "treemodel.h"
#include "treeitem.h"
#include "element.h"
#include "parameter.h"
#include "property_widget.h"
//#include <QKeyEvent>
//#include <QApplication>
//#include <iostream>
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
  PropertyDialog *editor = static_cast<PropertyDialog*>(model->getItem(index)->getItemData()->createPropertyDialog());
  editor->setModal(true);
  connect(editor,SIGNAL(apply(QWidget*)),this,SIGNAL(commitData(QWidget*)));
  connect(editor,SIGNAL(cancel(QWidget*)),this,SIGNAL(closeEditor(QWidget*)));
  connect(editor,SIGNAL(ok(QWidget*)),this,SLOT(commitDataAndcloseEditor(QWidget*)));
  return editor;
}

void ElementDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
  const ElementTreeModel *model = static_cast<const ElementTreeModel*>(index.model());
  PropertyDialog *dialog = static_cast<PropertyDialog*>(editor);
  dialog->toWidget(static_cast<Element*>(model->getItem(index)->getItemData()));
}

void ElementDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {
  PropertyDialog *dialog = static_cast<PropertyDialog*>(editor);
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
  PropertyDialog *editor = model->getItem(index)->getItemData()->createPropertyDialog();
  editor->setModal(true);
  connect(editor,SIGNAL(apply(QWidget*)),this,SIGNAL(commitData(QWidget*)));
  connect(editor,SIGNAL(cancel(QWidget*)),this,SIGNAL(closeEditor(QWidget*)));
  connect(editor,SIGNAL(ok(QWidget*)),this,SLOT(commitDataAndcloseEditor(QWidget*)));
  return editor;
}

void ParameterDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
  const ParameterListModel *model = static_cast<const ParameterListModel*>(index.model());
  PropertyDialog *dialog = static_cast<PropertyDialog*>(editor);
  dialog->toWidget(static_cast<Parameter*>(model->getItem(index)->getItemData()));
}

void ParameterDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {
  PropertyDialog *dialog = static_cast<PropertyDialog*>(editor);
  dialog->fromWidget(static_cast<Parameter*>(static_cast<const ParameterListModel*>(model)->getItem(index)->getItemData()));
  static_cast<ParameterListModel*>(model)->updateView(index);
  mw->mbsimxml(1);
}

void ParameterDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  QAbstractItemDelegate::updateEditorGeometry(editor,option,index);
}
