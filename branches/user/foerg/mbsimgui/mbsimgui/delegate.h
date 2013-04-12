#ifndef _DELEGATE_H
#define _DELEGATE_H

#include <QItemDelegate>
#include <QDialog>
#include <QLineEdit>

class ElementDelegate : public QItemDelegate {
  Q_OBJECT

  public:
    ElementDelegate(QObject *parent = 0) : QItemDelegate(parent) {}

    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const;

    void setEditorData(QWidget *editor, const QModelIndex &index) const;
    void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const;

    void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const;

  protected slots:
    void commitDataAndcloseEditor(QWidget *editor);
};

class ParameterDelegate : public QItemDelegate {
  Q_OBJECT

  public:
    ParameterDelegate(QObject *parent = 0) : QItemDelegate(parent) {} 

    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const;

    void setEditorData(QWidget *editor, const QModelIndex &index) const;
    void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const;

    void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const;

  protected slots:
    void commitDataAndcloseEditor(QWidget *editor);
};

#endif
