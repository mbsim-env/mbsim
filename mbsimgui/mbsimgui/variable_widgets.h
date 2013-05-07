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

class OctaveHighlighter : public QSyntaxHighlighter {

  public:
    OctaveHighlighter(QTextDocument *parent);

  protected:
    void highlightBlock(const QString &text);
    std::vector<std::pair<QRegExp, QTextCharFormat> > rule;
};

class VariableWidget : public Widget {

  public:
    virtual void setReadOnly(bool flag) {}
    virtual QString getValue() const = 0;
    virtual void setValue(const QString &str) = 0;
    virtual VariableWidget* cloneVariableWidget() {return 0;}
    virtual QString getType() const = 0;
    virtual bool validate(const QString &str) const {return true;}
};

class BoolWidget : public VariableWidget {

  public:
    BoolWidget(const QString &b="0");
    QString getValue() const {return value->checkState()==Qt::Checked?"1":"0";}
    void setValue(const QString &str) {value->setCheckState((str=="0"||str=="false")?Qt::Unchecked:Qt::Checked);}
    virtual VariableWidget* cloneVariableWidget() {return new BoolWidget;}
    virtual QString getType() const {return "Boolean";}

  protected:
    QCheckBox *value;
};

class OctaveExpressionWidget : public VariableWidget {
  public:
    OctaveExpressionWidget();
    QString getValue() const { return value->toPlainText(); }
    void setValue(const QString &str) { value->setPlainText(str); }
    virtual QString getType() const {return "Editor";}

  private:
    QPlainTextEdit *value;
};

class ScalarWidget : public VariableWidget {
  private:
    QLineEdit* box;
  public:
    ScalarWidget(const QString &d="1");
    void setReadOnly(bool flag) {box->setReadOnly(flag);}
    QString getValue() const {return box->text();}
    void setValue(const QString &str) {box->setText(str);}
    virtual VariableWidget* cloneVariableWidget() {return new ScalarWidget;}
    virtual QString getType() const {return "Scalar";}
};

class BasicVecWidget : public VariableWidget {
  public:
    virtual std::vector<QString> getVec() const = 0;
    virtual void setVec(const std::vector<QString> &x) = 0;
};

class VecWidget : public BasicVecWidget {
  private:
    std::vector<QLineEdit*> box;
    bool transpose;
  public:
    VecWidget(int size, bool transpose=false);
    VecWidget(const std::vector<QString> &x, bool transpose=false);
    void resize(int size);
    std::vector<QString> getVec() const;
    void setVec(const std::vector<QString> &x);
    void setReadOnly(bool flag);
    QString getValue() const {return toQStr(getVec());}
    void setValue(const QString &str) {setVec(strToVec(str));}
    int size() const {return box.size();}
    virtual VariableWidget* cloneVariableWidget() {return new VecWidget(size());}
    virtual QString getType() const {return "Vector";}
    bool validate(const QString &str) const;
};

class BasicMatWidget : public VariableWidget {
  public:
    virtual std::vector<std::vector<QString> > getMat() const = 0;
    virtual void setMat(const std::vector<std::vector<QString> > &A) = 0;
};

class MatWidget : public BasicMatWidget {

  private:
    std::vector<std::vector<QLineEdit*> > box;
  public:
    MatWidget(int rows, int cols);
    MatWidget(const std::vector<std::vector<QString> > &A);
    void resize(int rows, int cols);
    std::vector<std::vector<QString> > getMat() const;
    void setMat(const std::vector<std::vector<QString> > &A);
    void setReadOnly(bool flag);
    QString getValue() const {return toQStr(getMat());}
    void setValue(const QString &str) {setMat(strToMat(str));}
    int rows() const {return box.size();}
    int cols() const {return box[0].size();}
    virtual VariableWidget* cloneVariableWidget() {return new MatWidget(rows(),cols());}
    virtual QString getType() const {return "Matrix";}
    bool validate(const QString &str) const;
};

class SymMatWidget : public BasicMatWidget {

  private:
    std::vector<std::vector<QLineEdit*> > box;
  public:
    SymMatWidget(int rows);
    SymMatWidget(const std::vector<std::vector<QString> > &A);
    void resize(int rows);
    std::vector<std::vector<QString> > getMat() const;
    void setMat(const std::vector<std::vector<QString> > &A);
    void setReadOnly(bool flag);
    QString getValue() const {return toQStr(getMat());}
    void setValue(const QString &str) {setMat(strToMat(str));}
    virtual VariableWidget* cloneVariableWidget() {return new SymMatWidget(rows());}
    int rows() const {return box.size();}
    int cols() const {return box[0].size();}
    virtual QString getType() const {return "Matrix";}
    bool validate(const QString &str) const;
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
    void setVec(const std::vector<QString> &x) {
      sizeCombo->setValue(x.size());
      widget->setVec(x);
    }
    void resize(int size) {widget->resize(size);}
    int size() const {return sizeCombo->value();}
    QString getValue() const {return toQStr(getVec());}
    void setValue(const QString &str) {setVec(strToVec(str));}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual VariableWidget* cloneVariableWidget() {return new VecWidget(size());}
    virtual QString getType() const {return "Vector";}
    bool validate(const QString &str) const;

  public slots:
    void currentIndexChanged(int);
  signals:
    void sizeChanged(int);

};

class MatColsVarWidget : public BasicMatWidget {

  Q_OBJECT

  private:
    MatWidget *widget;
    QSpinBox* colsCombo;
    int minCols, maxCols;
  public:
    MatColsVarWidget(int rows, int cols, int minCols, int maxCols);
    std::vector<std::vector<QString> > getMat() const {return widget->getMat();}
    void setMat(const std::vector<std::vector<QString> > &A) {
      colsCombo->setValue(A[0].size());
      widget->setMat(A);
    }
    void resize(int rows, int cols) {widget->resize(rows,cols);}
    int rows() const {return widget->rows();}
    int cols() const {return colsCombo->value();}
    QString getValue() const {return toQStr(getMat());}
    void setValue(const QString &str) {setMat(strToMat(str));}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual VariableWidget* cloneVariableWidget() {return new MatWidget(rows(),cols());}
    virtual QString getType() const {return "Matrix";}
    bool validate(const QString &str) const;

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
    void setMat(const std::vector<std::vector<QString> > &A) {
      rowsCombo->setValue(A.size());
      colsCombo->setValue(A[0].size());
      widget->setMat(A);
    }
    void resize(int rows, int cols) {widget->resize(rows,cols);}
    int rows() const {return rowsCombo->value();}
    int cols() const {return colsCombo->value();}
    QString getValue() const {return toQStr(getMat());}
    void setValue(const QString &str) {setMat(strToMat(str));}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual VariableWidget* cloneVariableWidget() {return new MatWidget(rows(),cols());}
    virtual QString getType() const {return "Matrix";}
    bool validate(const QString &str) const;

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
    bool transpose;
  public:
    CardanWidget(bool transpose=false);
    CardanWidget(const std::vector<QString> &x, bool transpose=false);
    std::vector<QString> getCardan() const;
    void setCardan(const std::vector<QString> &x);
    void setReadOnly(bool flag);
    QString getValue() const {return toQStr(getCardan());}
    void setValue(const QString &str) {setCardan(strToVec(str));}
    virtual VariableWidget* cloneVariableWidget() {return new CardanWidget;}
    virtual QString getType() const {return "Cardan";}
};

class PhysicalVariableWidget : public VariableWidget {

  Q_OBJECT

  private:
    VariableWidget *widget;
    QComboBox* unit;
    QStringList units;
    int defaultUnit;
  public:
    PhysicalVariableWidget(VariableWidget *widget, const QStringList &units, int defaultUnit);
    QString getValue() const {return widget->getValue();}
    void setValue(const QString &str) {widget->setValue(str);}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual VariableWidget* cloneVariableWidget() {return widget->cloneVariableWidget();}
    virtual VariableWidget* getWidget() {return widget;}
    const QStringList& getUnitList() const {return units;}
    int getDefaultUnit() const {return defaultUnit;}
    virtual QString getType() const {return widget->getType();}
    bool validate(const QString &str) const {return widget->validate(str);}
    QString getUnit() const {return unit->currentText();}
    void setUnit(const QString &unit_) {unit->setCurrentIndex(unit->findText(unit_));}
};

class VecFromFileWidget : public VariableWidget {
  Q_OBJECT

  friend class VecFromFileProperty;

  public:
    VecFromFileWidget();
    QString getValue() const;
    void setValue(const QString &str) {}
    QString getFile() const {return file;}
    void setFile(const QString &str);
    virtual QString getType() const {return "File";}
    virtual VariableWidget* cloneVariableWidget() {return new VecWidget(0);}

  protected:
    QLineEdit *relativeFilePath;
    QString file; 

  protected slots:
    void selectFile();

};

class MatFromFileWidget : public VariableWidget {
  Q_OBJECT

  friend class MatFromFileProperty;

  public:
    MatFromFileWidget();
    QString getValue() const; 
    void setValue(const QString &str) {}
    QString getFile() const {return file;}
    void setFile(const QString &str);
    virtual QString getType() const {return "File";}
    virtual VariableWidget* cloneVariableWidget() {return new MatWidget(0,0);}

  protected:
    QLineEdit *relativeFilePath;
    QString file; 

  protected slots:
    void selectFile();

};


#endif

