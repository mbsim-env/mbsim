#ifndef delegate_h
#define delegate_h
#include <QStyledItemDelegate>
#include <QDialog>
#include <QLineEdit>

class Delegate : public QStyledItemDelegate {
  Q_OBJECT

  public:
    Delegate(QObject *parent = 0);

    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option,
        const QModelIndex &index) const;

    void setEditorData(QWidget *editor, const QModelIndex &index) const;
    void setModelData(QWidget *editor, QAbstractItemModel *model,
        const QModelIndex &index) const;

    void updateEditorGeometry(QWidget *editor,
        const QStyleOptionViewItem &option, const QModelIndex &index) const;
  protected slots:
    void commitDataAndcloseEditor(QWidget *editor);

  protected:
    bool eventFilter(QObject *editor, QEvent *event);
};

#endif
