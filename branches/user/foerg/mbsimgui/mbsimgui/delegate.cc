#include "delegate.h"
#include "treemodel.h"
#include "treeitem.h"
#include "element.h"
#include "property_widget.h"
#include <QSpinBox>
#include <QVBoxLayout>
#include <QKeyEvent>
#include <QApplication>
#include <iostream>
#include "mainwindow.h"

using namespace std;

extern MainWindow *mw;

Delegate::Delegate(QObject *parent) : QStyledItemDelegate(parent) {
//  QItemEditorFactory *factory = new QItemEditorFactory;
//
//  QItemEditorCreatorBase *elementEditor =
//    new QStandardItemEditorCreator<ElementPropertyDialog>();
//
//  factory->registerEditor(QVariant::String, elementEditor);
//
//  setItemEditorFactory(factory);
}

void Delegate::commitDataAndcloseEditor(QWidget *editor) {
  emit commitData(editor);
  emit closeEditor(editor);
}

bool Delegate::eventFilter(QObject *object, QEvent *event) {
  return false;
}

QWidget *Delegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  const TreeModel *model = static_cast<const TreeModel*>(index.model());
  PropertyDialog *editor = model->getItem(index)->getItemData()->createPropertyDialog();
  editor->setModal(true);
  connect(editor,SIGNAL(apply(QWidget*)),this,SIGNAL(commitData(QWidget*)));
  connect(editor,SIGNAL(cancel(QWidget*)),this,SIGNAL(closeEditor(QWidget*)));
  connect(editor,SIGNAL(ok(QWidget*)),this,SLOT(commitDataAndcloseEditor(QWidget*)));
  return editor;
}

void Delegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
  const TreeModel *model = static_cast<const TreeModel*>(index.model());
  PropertyDialog *dialog = static_cast<PropertyDialog*>(editor);
  dialog->toWidget(static_cast<Element*>(model->getItem(index)->getItemData()));
}

void Delegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {
  PropertyDialog *dialog = static_cast<PropertyDialog*>(editor);
  dialog->fromWidget(static_cast<Element*>(static_cast<const TreeModel*>(model)->getItem(index)->getItemData()));
  static_cast<TreeModel*>(model)->updateView(index);
  mw->mbsimxml(1);
}

void Delegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  QAbstractItemDelegate::updateEditorGeometry(editor,option,index);
}
