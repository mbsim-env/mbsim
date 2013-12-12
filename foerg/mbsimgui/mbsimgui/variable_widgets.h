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

#ifndef _STRING_WIDGETS_H_
#define _STRING_WIDGETS_H_

#include "widget.h"
#include "utils.h"
#include <QCheckBox>
#include <QComboBox>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <QLineEdit>
#include <QSyntaxHighlighter>
#include <QGridLayout>
#include "property_context_menu.h"

class EvalDialog;
class QLabel;
class VariableProperty;

class OctaveHighlighter : public QSyntaxHighlighter {

  public:
    OctaveHighlighter(QTextDocument *parent);

  protected:
    void highlightBlock(const QString &text);
    std::vector<std::pair<QRegExp, QTextCharFormat> > rule;
};

class VariableWidget : public Widget {

  public:
    VariableWidget(const Units &unit=NoUnitUnits()) : Widget(unit) { }
    virtual void setReadOnly(bool flag) {}
    virtual QString getValue() const {return "";}
    virtual void setValue(const QString &str) {}
    virtual QString getType() const = 0;
    virtual bool validate(const std::vector<std::vector<QString> > &A) const {return true;}
};

class BoolWidget : public VariableWidget {

  public:
    BoolWidget(const QString &b="0");
    QString getValue() const {return value->checkState()==Qt::Checked?"true":"false";}
    void setValue(const QString &str) {value->setCheckState((str=="0"||str=="false")?Qt::Unchecked:Qt::Checked);}
    virtual QString getType() const {return "Boolean";}
    bool validate(const std::vector<std::vector<QString> > &A) const;
    void fromProperty(Property *property);
    void toProperty(Property *property);

  protected:
    QCheckBox *value;
};

class OctaveExpressionWidget : public VariableWidget {
  public:
    OctaveExpressionWidget(const QString &value="", const Units &unit=NoUnitUnits());
    OctaveExpressionWidget(Property *property);
    QString getExpression() const { return value->toPlainText(); }
    void setExpression(const QString &str) { value->setPlainText(str); }
    virtual QString getType() const {return "Editor";}
    void fromProperty(Property *property);
    void toProperty(Property *property);

  private:
    QPlainTextEdit *value;
    Property *property;
};

class ScalarWidget : public VariableWidget {
  private:
    QLineEdit* box;
    Property *property;
  public:
    ScalarWidget(const QString &d="1", const Units &unit=NoUnitUnits());
    ScalarWidget(Property *property);
    void setReadOnly(bool flag) {box->setReadOnly(flag);}
    const QString getScalar() const {return box->text().isEmpty()?"0":box->text();}
    void setScalar(const QString &str) {box->setText(str=="0"?"":str);}
    virtual QString getType() const {return "Scalar";}
    bool validate(const std::vector<std::vector<QString> > &A) const;
    void fromProperty(Property *property);
    void toProperty(Property *property);
};

class BasicVecWidget : public VariableWidget {
  protected:
    Property *property;
  public:
    BasicVecWidget(const Units &unit=NoUnitUnits()) : VariableWidget(unit), property(0) { }
    BasicVecWidget(Property *property_, const Units &unit=NoUnitUnits()) : VariableWidget(unit), property(property_) { }
    virtual std::vector<QString> getVec() const = 0;
    virtual void setVec(const std::vector<QString> &x) = 0;
    void fromProperty(Property *property);
    void toProperty(Property *property);
};

class VecWidget : public BasicVecWidget {
  private:
    //QGridLayout *layout;
    std::vector<QLineEdit*> box;
    bool transpose;
  public:
    VecWidget(int size=3, bool transpose=false, const Units &unit=NoUnitUnits());
    VecWidget(const std::vector<QString> &x, bool transpose=false, const Units &unit=NoUnitUnits());
    VecWidget(Property *property);
    void resize_(int size);
    void resize_(int rows, int cols) { resize_(rows); }
    std::vector<QString> getVec() const;
    void setVec(const std::vector<QString> &x);
    void setReadOnly(bool flag);
    QString getValue() const {return toQStr(getVec());}
    void setValue(const QString &str) {setVec(strToVec(str));}
    int size() const {return box.size();}
    virtual QString getType() const {return "Vector";}
    bool validate(const std::vector<std::vector<QString> > &A) const;
};

class BasicMatWidget : public VariableWidget {
  protected:
    Property *property;
  public:
    BasicMatWidget(const Units &unit=NoUnitUnits()) : VariableWidget(unit), property(0) { }
    BasicMatWidget(Property *property_, const Units &unit=NoUnitUnits()) : VariableWidget(unit), property(property_) { }
    virtual std::vector<std::vector<QString> > getMat() const = 0;
    virtual void setMat(const std::vector<std::vector<QString> > &A) = 0;
    void fromProperty(Property *property);
    void toProperty(Property *property);
};

class MatWidget : public BasicMatWidget {

  private:
    std::vector<std::vector<QLineEdit*> > box;
  public:
    MatWidget(int rows=1, int cols=1, const Units &unit=NoUnitUnits());
    MatWidget(const std::vector<std::vector<QString> > &A, const Units &unit=NoUnitUnits());
    MatWidget(Property *property); 
    void resize_(int rows, int cols);
    std::vector<std::vector<QString> > getMat() const;
    void setMat(const std::vector<std::vector<QString> > &A);
    void setReadOnly(bool flag);
    QString getValue() const {return toQStr(getMat());}
    void setValue(const QString &str) {setMat(strToMat(str));}
    int rows() const {return box.size();}
    int cols() const {return box[0].size();}
    virtual QString getType() const {return "Matrix";}
    bool validate(const std::vector<std::vector<QString> > &A) const;
};

class SymMatWidget : public BasicMatWidget {

  private:
    std::vector<std::vector<QLineEdit*> > box;
  public:
    SymMatWidget(int rows, const Units &unit=NoUnitUnits());
    SymMatWidget(const std::vector<std::vector<QString> > &A, const Units &unit=NoUnitUnits());
    SymMatWidget(Property *property); 
    void resize_(int rows);
    std::vector<std::vector<QString> > getMat() const;
    void setMat(const std::vector<std::vector<QString> > &A);
    void setReadOnly(bool flag);
    QString getValue() const {return toQStr(getMat());}
    void setValue(const QString &str) {setMat(strToMat(str));}
    int rows() const {return box.size();}
    int cols() const {return box[0].size();}
    virtual QString getType() const {return "Matrix";}
    bool validate(const std::vector<std::vector<QString> > &A) const;
};

class VecSizeVarWidget : public BasicVecWidget {

  Q_OBJECT

  private:
    VecWidget *widget;
    QSpinBox* sizeCombo;
    int minSize, maxSize;
  public:
    VecSizeVarWidget(int size, int minSize, int maxSize);
    std::vector<QString> getVec() const {return widget->getVec();}
    void setVec(const std::vector<QString> &x);
    void resize_(int size);
    int size() const {return sizeCombo->value();}
    QString getValue() const {return toQStr(getVec());}
    void setValue(const QString &str) {setVec(strToVec(str));}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual QString getType() const {return "Vector";}
    bool validate(const std::vector<std::vector<QString> > &A) const;

  public slots:
    void currentIndexChanged(int);
  signals:
    void sizeChanged(int);

};

class MatColsVarWidget : public BasicMatWidget {

  Q_OBJECT

  private:
    MatWidget *widget;
    QLabel *rowsLabel;
    QSpinBox* colsCombo;
    int minCols, maxCols;
  public:
    MatColsVarWidget(Property *property=0, int rows=3, int cols=1, int minCols=1, int maxCols=3);
    std::vector<std::vector<QString> > getMat() const {return widget->getMat();}
    void setMat(const std::vector<std::vector<QString> > &A);
    void resize_(int rows, int cols);
    int rows() const {return widget->rows();}
    int cols() const {return colsCombo->value();}
    QString getValue() const {return toQStr(getMat());}
    void setValue(const QString &str) {setMat(strToMat(str));}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual QString getType() const {return "Matrix";}
    bool validate(const std::vector<std::vector<QString> > &A) const;

  public slots:
    void currentIndexChanged(int);
  signals:
    void sizeChanged(int);
};

class MatRowsVarWidget : public BasicMatWidget {

  Q_OBJECT

  private:
    MatWidget *widget;
    QSpinBox *rowsCombo;
    QLabel *colsLabel;
    int minRows, maxRows;
  public:
    MatRowsVarWidget(int rows, int cols, int minRows, int maxRows);
    std::vector<std::vector<QString> > getMat() const {return widget->getMat();}
    void setMat(const std::vector<std::vector<QString> > &A);
    void resize_(int rows, int cols);
    int rows() const {return rowsCombo->value();}
    int cols() const {return widget->cols();}
    QString getValue() const {return toQStr(getMat());}
    void setValue(const QString &str) {setMat(strToMat(str));}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual QString getType() const {return "Matrix";}
    bool validate(const std::vector<std::vector<QString> > &A) const;

  public slots:
    void currentIndexChanged(int);
  signals:
    void sizeChanged(int);
};

class MatRowsColsVarWidget : public BasicMatWidget {

  Q_OBJECT

  private:
    MatWidget *widget;
    QSpinBox *rowsCombo, *colsCombo;
    int minRows, maxRows, minCols, maxCols;
  public:
    MatRowsColsVarWidget(int rows, int cols, int minRows, int maxRows, int minCols, int maxCols);
    std::vector<std::vector<QString> > getMat() const {return widget->getMat();}
    void setMat(const std::vector<std::vector<QString> > &A);
    void resize_(int rows, int cols);
    int rows() const {return rowsCombo->value();}
    int cols() const {return colsCombo->value();}
    QString getValue() const {return toQStr(getMat());}
    void setValue(const QString &str) {setMat(strToMat(str));}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual QString getType() const {return "Matrix";}
    bool validate(const std::vector<std::vector<QString> > &A) const;

  public slots:
    void currentRowIndexChanged(int);
    void currentColIndexChanged(int);
  signals:
    void rowSizeChanged(int);
    void colSizeChanged(int);
};

class CardanWidget : public VariableWidget {

  private:
    std::vector<QLineEdit*> box;
    Property *property;
  public:
    CardanWidget(Property *property=0);
    void resize_(int size);
    std::vector<QString> getAngles() const;
    void setAngles(const std::vector<QString> &x);
    void setReadOnly(bool flag);
    QString getValue() const {return toQStr(getAngles());}
    void setValue(const QString &str) {setAngles(strToVec(str));}
    int size() const {return box.size();}
    virtual QString getType() const {return "Cardan";}
    bool validate(const std::vector<std::vector<QString> > &A) const;
    void fromProperty(Property *property);
    void toProperty(Property *property);
};

class PhysicalVariableWidget : public VariableWidget {

  Q_OBJECT

  private:
    VariableWidget *widget;
    QComboBox* unit;
    QStringList units;
    int defaultUnit;
    EvalDialog *evalDialog;
  protected slots:
    void openEvalDialog();
  public:
    PhysicalVariableWidget(VariableWidget *widget, const QStringList &units, int defaultUnit);
    QString getValue() const {return widget->getValue();}
    void setValue(const QString &str) {widget->setValue(str);}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual VariableWidget* getWidget() {return widget;}
    const QStringList& getUnitList() const {return units;}
    int getDefaultUnit() const {return defaultUnit;}
    virtual QString getType() const {return widget->getType();}
    bool validate(const std::vector<std::vector<QString> > &A) const {return widget->validate(A);}
    QString getUnit() const {return unit->currentText();}
    void setUnit(const QString &unit_) {unit->setCurrentIndex(unit->findText(unit_));}
    void resize_(int rows, int cols) { widget->resize_(rows,cols); }
};

//class VecFromFileWidget : public VariableWidget {
//  Q_OBJECT
//
//  friend class VecFromFileProperty;
//
//  public:
//    VecFromFileWidget();
//    QString getValue() const;
//    void setValue(const QString &str) {}
//    QString getFile() const {return file;}
//    void setFile(const QString &str);
//    virtual QString getType() const {return "File";}
//
//  protected:
//    QLineEdit *relativeFilePath;
//    QString file; 
//
//  protected slots:
//    void selectFile();
//
//};

class FromFileWidget : public VariableWidget {
  Q_OBJECT

  friend class FromFileProperty;

  public:
    FromFileWidget();
    QString getValue() const; 
    void setValue(const QString &str) {}
    QString getFile() const {return relativeFilePath->text();}
    void setFile(const QString &str);
    virtual QString getType() const {return "File";}

  protected:
    QLineEdit *relativeFilePath;
//    QString file; 

  protected slots:
    void selectFile();

};

class ScalarWidgetFactory : public WidgetFactory {
  public:
    ScalarWidgetFactory(const QString &value, const std::vector<QString> &name, const Units &unit=NoUnitUnits());
    ScalarWidgetFactory(const QString &value="0", const Units &unit=NoUnitUnits());
    QWidget* createWidget(int i=0);
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    QString value;
    std::vector<QString> name;
    Units unit;
};

class VecWidgetFactory : public WidgetFactory {
  public:
    VecWidgetFactory(int m, const std::vector<QString> &name, const Units &unit=NoUnitUnits());
    VecWidgetFactory(int m, const Units &unit=NoUnitUnits());
    QWidget* createWidget(int i=0);
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    int m;
    std::vector<QString> name;
    Units unit;
};

class VecSizeVarWidgetFactory : public WidgetFactory {
  public:
    VecSizeVarWidgetFactory(int m);
    VecSizeVarWidgetFactory(int m, const std::vector<QString> &name, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
    VecSizeVarWidgetFactory(int m, const std::vector<QStringList> &unit);
    QWidget* createWidget(int i=0);
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    int m;
    std::vector<QString> name;
    std::vector<QStringList> unit;
    std::vector<int> defaultUnit;
};

class MatWidgetFactory : public WidgetFactory {
  public:
    MatWidgetFactory(const Units &unit=NoUnitUnits());
    MatWidgetFactory(const std::vector<std::vector<QString> > &A, const Units &unit=NoUnitUnits());
    MatWidgetFactory(const std::vector<std::vector<QString> > &A, const std::vector<QString> &name, const Units &unit=NoUnitUnits());
    QWidget* createWidget(int i=0);
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<std::vector<QString> > A;
    std::vector<QString> name;
    Units unit;
};

class MatRowsVarWidgetFactory : public WidgetFactory {
  public:
    MatRowsVarWidgetFactory();
    MatRowsVarWidgetFactory(const std::vector<std::vector<QString> > &A, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
    MatRowsVarWidgetFactory(const std::vector<std::vector<QString> > &A, const std::vector<QString> &name, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
    QWidget* createWidget(int i=0);
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<std::vector<QString> > A;
    std::vector<QString> name;
    std::vector<QStringList> unit;
    std::vector<int> defaultUnit;
};

class RotMatWidgetFactory : public WidgetFactory {
  public:
    RotMatWidgetFactory();
    RotMatWidgetFactory(const std::vector<QString> &name);
    QWidget* createWidget(int i=0);
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<QString> name;
};

class SymMatWidgetFactory : public MatWidgetFactory {
  public:
    SymMatWidgetFactory(const Units &unit=NoUnitUnits()) : MatWidgetFactory(unit) { }
    SymMatWidgetFactory(const std::vector<std::vector<QString> > &A, const Units &unit=NoUnitUnits()) : MatWidgetFactory(A,unit) { }
    SymMatWidgetFactory(const std::vector<std::vector<QString> > &A, const std::vector<QString> &name, const Units &unit=NoUnitUnits()) : MatWidgetFactory(A,name,unit) { }
    QWidget* createWidget(int i=0);
};

class VariableChoiceContextMenu : public PropertyContextMenu {

  Q_OBJECT
  public:
    VariableChoiceContextMenu(VariableProperty *property, QWidget * parent = 0, bool removable=false);
  protected:
    std::map<QAction*,int> actions;
  protected slots:
    void setVariable(QAction*);
};

#endif

