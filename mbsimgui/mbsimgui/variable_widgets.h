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
#include "custom_widgets.h"
#include <QCheckBox>
#include <QPlainTextEdit>
#include <QLineEdit>
#include <QSyntaxHighlighter>

class QLabel;
class QTableWidget;

namespace MBSimGUI {

  class OctaveHighlighter : public QSyntaxHighlighter {

    public:
      OctaveHighlighter(QTextDocument *parent);

    protected:
      void highlightBlock(const QString &text) override;
      std::vector<std::pair<QRegExp, QTextCharFormat> > rule;
  };

  class VariableWidget : public Widget {

    public:
      virtual void setReadOnly(bool flag) {}
      virtual QString getValue() const = 0;
      virtual void setValue(const QString &str) = 0;
      virtual QString getType() const = 0;
      virtual bool validate(const std::vector<std::vector<QString> > &A) const {return true;}
      virtual int rows() const { return 1; }
      virtual int cols() const { return 1; }
      virtual std::vector<std::vector<QString> > getEvalMat() const;
  };

  class BoolWidget : public VariableWidget {

    public:
      BoolWidget(const QString &b="0");
      QString getValue() const override {return value->checkState()==Qt::Checked?"true":"false";}
      void setValue(const QString &str) override {value->setCheckState((str=="0"||str=="false")?Qt::Unchecked:Qt::Checked);}
      QString getType() const override {return "Boolean";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;
      std::vector<std::vector<QString> > getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QCheckBox *value;
  };

  class ExpressionWidget : public VariableWidget {
    public:
      ExpressionWidget(const QString &str="");
      QString getValue() const override { return value->toPlainText(); }
      void setValue(const QString &str) override { value->setPlainText(str); }
      QString getType() const override {return "Editor";}
      int rows() const override { return getEvalMat().size(); }
      int cols() const override { return !getEvalMat().empty()?getEvalMat()[0].size():0; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    private:
      QPlainTextEdit *value;
  };

  class ScalarWidget : public VariableWidget {
    private:
      QLineEdit* box;
    public:
      ScalarWidget(const QString &d="1");
      void setReadOnly(bool flag) override {box->setReadOnly(flag);}
      QString getValue() const override {return box->text().isEmpty()?"0":box->text();}
      void setValue(const QString &str) override {box->setText(str=="0"?"":str);}
      QString getType() const override {return "Scalar";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;
      std::vector<std::vector<QString> > getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class BasicVecWidget : public VariableWidget {
    public:
      virtual std::vector<QString> getVec() const = 0;
      virtual void setVec(const std::vector<QString> &x) = 0;
      std::vector<std::vector<QString> > getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class VecWidget : public BasicVecWidget {
    private:
      std::vector<QLineEdit*> box;
      bool transpose;
    public:
      VecWidget(int size, bool transpose_=false);
      VecWidget(const std::vector<QString> &x, bool transpose_=false);
      void resize_(int size);
      void resize_(int rows, int cols) override { resize_(rows); }
      std::vector<QString> getVec() const override;
      void setVec(const std::vector<QString> &x) override;
      void setReadOnly(bool flag) override;
      QString getValue() const override {return toQStr(getVec());}
      void setValue(const QString &str) override {setVec(strToVec(str));}
      int size() const {return box.size();}
      int rows() const override { return size(); }
      QString getType() const override {return "Vector";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;
  };


  class VecSizeVarWidget : public BasicVecWidget {

    Q_OBJECT

    private:
      BasicVecWidget *widget;
      CustomSpinBox* sizeCombo;
      int minSize, maxSize;
    public:
      VecSizeVarWidget(int size, int minSize_, int maxSize_, int singleStep=1, bool transpose=false, bool table=false);
//      VecSizeVarWidget(const std::vector<QString> &x, int minSize, int maxSize, bool transpose=false);
      std::vector<QString> getVec() const override {return widget->getVec();}
      void setVec(const std::vector<QString> &x) override;
      void resize_(int size);
      int size() const {return sizeCombo->value();}
      int rows() const override { return size(); }
      int cols() const override { return 1; }
      QString getValue() const override {return toQStr(getVec());}
      void setValue(const QString &str) override {setVec(strToVec(str));}
      void setReadOnly(bool flag) override {widget->setReadOnly(flag);}
      QString getType() const override {return "Vector";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;

    public slots:
      void currentIndexChanged(int);
    signals:
      void sizeChanged(int);
  };

  class VecTableWidget: public BasicVecWidget {

    private:
      QTableWidget *table;
    public:
      VecTableWidget(int size);
      VecTableWidget(const std::vector<QString> &x);
      void resize_(int size);
      void resize_(int rows, int cols) override { resize_(rows); }
      std::vector<QString> getVec() const override;
      void setVec(const std::vector<QString> &x) override;
//      void setReadOnly(bool flag);
      QString getValue() const override {return toQStr(getVec());}
      void setValue(const QString &str) override {setVec(strToVec(str));}
      int size() const;
      int rows() const override { return size(); }
      QString getType() const override {return "Vector";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;
  };

  class BasicMatWidget : public VariableWidget {
    public:
      virtual std::vector<std::vector<QString> > getMat() const = 0;
      virtual void setMat(const std::vector<std::vector<QString> > &A) = 0;
      std::vector<std::vector<QString> > getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class MatWidget : public BasicMatWidget {

    private:
      std::vector<std::vector<QLineEdit*> > box;
    public:
      MatWidget(int rows, int cols);
      MatWidget(const std::vector<std::vector<QString> > &A);
      void resize_(int rows, int cols) override;
      std::vector<std::vector<QString> > getMat() const override;
      void setMat(const std::vector<std::vector<QString> > &A) override;
      void setReadOnly(bool flag) override;
      QString getValue() const override {return toQStr(getMat());}
      void setValue(const QString &str) override {setMat(strToMat(str));}
      int rows() const override {return box.size();}
      int cols() const override {return box[0].size();}
      QString getType() const override {return "Matrix";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;
  };

  class MatColsVarWidget : public BasicMatWidget {

    Q_OBJECT

    private:
      BasicMatWidget *widget;
      QLabel *rowsLabel;
      CustomSpinBox* colsCombo;
      int minCols, maxCols;
    public:
      MatColsVarWidget(int rows, int cols, int minCols_, int maxCols_, int table=false);
      std::vector<std::vector<QString> > getMat() const override {return widget->getMat();}
      void setMat(const std::vector<std::vector<QString> > &A) override;
      void resize_(int rows, int cols) override;
      int rows() const override {return widget->rows();}
      int cols() const override {return colsCombo->value();}
      QString getValue() const override {return toQStr(getMat());}
      void setValue(const QString &str) override {setMat(strToMat(str));}
      void setReadOnly(bool flag) override {widget->setReadOnly(flag);}
      QString getType() const override {return "Matrix";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;

    public slots:
      void currentIndexChanged(int);
    signals:
      void sizeChanged(int);
  };

  class MatRowsVarWidget : public BasicMatWidget {

    Q_OBJECT

    private:
      BasicMatWidget *widget;
      CustomSpinBox *rowsCombo;
      QLabel *colsLabel;
      int minRows, maxRows;
    public:
      MatRowsVarWidget(int rows, int cols, int minRows_, int maxRows_, int table=false);
      std::vector<std::vector<QString> > getMat() const override {return widget->getMat();}
      void setMat(const std::vector<std::vector<QString> > &A) override;
      void resize_(int rows, int cols) override;
      int rows() const override {return rowsCombo->value();}
      int cols() const override {return widget->cols();}
      QString getValue() const override {return toQStr(getMat());}
      void setValue(const QString &str) override {setMat(strToMat(str));}
      void setReadOnly(bool flag) override {widget->setReadOnly(flag);}
      QString getType() const override {return "Matrix";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;

    public slots:
      void currentIndexChanged(int);
    signals:
      void sizeChanged(int);
  };

  class MatRowsColsVarWidget : public BasicMatWidget {

    Q_OBJECT

    private:
      BasicMatWidget *widget;
      CustomSpinBox *rowsCombo, *colsCombo;
      int minRows, maxRows, minCols, maxCols;
    public:
      MatRowsColsVarWidget(int rows, int cols, int minRows_, int maxRows_, int minCols_, int maxCols_, int table=false);
      std::vector<std::vector<QString> > getMat() const override {return widget->getMat();}
      void setMat(const std::vector<std::vector<QString> > &A) override;
      void resize_(int rows, int cols) override;
      int rows() const override {return rowsCombo->value();}
      int cols() const override {return colsCombo->value();}
      QString getValue() const override {return toQStr(getMat());}
      void setValue(const QString &str) override {setMat(strToMat(str));}
      void setReadOnly(bool flag) override {widget->setReadOnly(flag);}
      QString getType() const override {return "Matrix";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;

    public slots:
      void currentRowIndexChanged(int);
      void currentColIndexChanged(int);
    signals:
      void rowSizeChanged(int);
      void colSizeChanged(int);
  };

  class SqrMatSizeVarWidget : public BasicMatWidget {

    Q_OBJECT

    private:
      BasicMatWidget *widget;
      CustomSpinBox *sizeCombo;
      int minSize, maxSize;
    public:
      SqrMatSizeVarWidget(int size, int minSize_, int maxSize_);
      std::vector<std::vector<QString> > getMat() const override {return widget->getMat();}
      void setMat(const std::vector<std::vector<QString> > &A) override;
      void resize_(int rows, int cols) override;
      int rows() const override {return sizeCombo->value();}
      int cols() const override {return rows();}
      QString getValue() const override {return toQStr(getMat());}
      void setValue(const QString &str) override {setMat(strToMat(str));}
      void setReadOnly(bool flag) override {widget->setReadOnly(flag);}
      QString getType() const override {return "Matrix";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;

    public slots:
      void currentIndexChanged(int);
    signals:
      void sizeChanged(int);
  };

  class SymMatWidget : public BasicMatWidget {

    private:
      std::vector<std::vector<QLineEdit*> > box;
    public:
      SymMatWidget(int rows);
      SymMatWidget(const std::vector<std::vector<QString> > &A);
      void resize_(int rows);
      void resize_(int rows, int cols) override { resize_(rows); }
      std::vector<std::vector<QString> > getMat() const override;
      void setMat(const std::vector<std::vector<QString> > &A) override;
      void setReadOnly(bool flag) override;
      QString getValue() const override {return toQStr(getMat());}
      void setValue(const QString &str) override {setMat(strToMat(str));}
      int rows() const override {return box.size();}
      int cols() const override {return box[0].size();}
      QString getType() const override {return "Matrix";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;
  };

  class SymMatSizeVarWidget : public BasicMatWidget {

    Q_OBJECT

    private:
      SymMatWidget *widget;
      CustomSpinBox *sizeCombo;
      int minSize, maxSize;
    public:
      SymMatSizeVarWidget(int size, int minSize_, int maxSize_);
      std::vector<std::vector<QString> > getMat() const override {return widget->getMat();}
      void setMat(const std::vector<std::vector<QString> > &A) override;
      void resize_(int rows, int cols) override;
      int rows() const override {return sizeCombo->value();}
      int cols() const override {return rows();}
      QString getValue() const override {return toQStr(getMat());}
      void setValue(const QString &str) override {setMat(strToMat(str));}
      void setReadOnly(bool flag) override {widget->setReadOnly(flag);}
      QString getType() const override {return "Matrix";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;

    public slots:
      void currentIndexChanged(int);
    signals:
      void sizeChanged(int);
  };

  class MatTableWidget: public BasicMatWidget {

    private:
      QTableWidget *table;
    public:
      MatTableWidget(int rows, int cols);
      MatTableWidget(const std::vector<std::vector<QString> > &A);
      void resize_(int rows, int cols) override;
      std::vector<std::vector<QString> > getMat() const override;
      void setMat(const std::vector<std::vector<QString> > &A) override;
//      void setReadOnly(bool flag);
      QString getValue() const override {return toQStr(getMat());}
      void setValue(const QString &str) override {setMat(strToMat(str));}
      int rows() const override;
      int cols() const override;
      QString getType() const override {return "Matrix";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;
  };

  class CardanWidget : public VariableWidget {

    private:
      std::vector<QLineEdit*> box;
      QComboBox* unit;
    public:
      CardanWidget();
      std::vector<QString> getAngles() const;
      void setAngles(const std::vector<QString> &x);
      void setReadOnly(bool flag) override;
      QString getValue() const override {return toQStr(getAngles());}
      void setValue(const QString &str) override {setAngles(strToVec(str));}
      int size() const {return box.size();}
      QString getType() const override {return "Cardan";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;
      QString getUnit() const {return unit->currentText();}
      void setUnit(const QString &unit_) {unit->setCurrentIndex(unit->findText(unit_));}
      std::vector<std::vector<QString> > getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class AboutXWidget : public VariableWidget {

    private:
      QLineEdit* box;
      QComboBox* unit;
    public:
      AboutXWidget();
      QString getValue() const override {return box->text().isEmpty()?"0":box->text();}
      void setValue(const QString &str) override {box->setText(str=="0"?"":str);}
      QString getType() const override {return "AboutX";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;
      QString getUnit() const {return unit->currentText();}
      void setUnit(const QString &unit_) {unit->setCurrentIndex(unit->findText(unit_));}
      std::vector<std::vector<QString> > getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class AboutYWidget : public VariableWidget {

    private:
      QLineEdit* box;
      QComboBox* unit;
    public:
      AboutYWidget();
      QString getValue() const override {return box->text().isEmpty()?"0":box->text();}
      void setValue(const QString &str) override {box->setText(str=="0"?"":str);}
      QString getType() const override {return "AboutY";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;
      QString getUnit() const {return unit->currentText();}
      void setUnit(const QString &unit_) {unit->setCurrentIndex(unit->findText(unit_));}
      std::vector<std::vector<QString> > getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class AboutZWidget : public VariableWidget {

    private:
      QLineEdit* box;
      QComboBox* unit;
    public:
      AboutZWidget();
      QString getValue() const override {return box->text().isEmpty()?"0":box->text();}
      void setValue(const QString &str) override {box->setText(str=="0"?"":str);}
      QString getType() const override {return "AboutZ";}
      bool validate(const std::vector<std::vector<QString> > &A) const override;
      QString getUnit() const {return unit->currentText();}
      void setUnit(const QString &unit_) {unit->setCurrentIndex(unit->findText(unit_));}
      std::vector<std::vector<QString> > getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class PhysicalVariableWidget : public VariableWidget {

    Q_OBJECT

    private:
      VariableWidget *widget;
      QComboBox* unit;
      QStringList units;
      int defaultUnit;
    protected slots:
      void openEvalDialog();
    public:
      PhysicalVariableWidget(VariableWidget *widget_, const QStringList &units_=QStringList(), int defaultUnit_=0, bool eval=true);
      QString getValue() const override {return widget->getValue();}
      void setValue(const QString &str) override {widget->setValue(str);}
      void setReadOnly(bool flag) override {widget->setReadOnly(flag);}
      virtual VariableWidget* getWidget() {return widget;}
      const QStringList& getUnitList() const {return units;}
      int getDefaultUnit() const {return defaultUnit;}
      QString getType() const override {return widget->getType();}
      bool validate(const std::vector<std::vector<QString> > &A) const override {return widget->validate(A);}
      QString getUnit() const {return unit->currentText();}
      void setUnit(const QString &unit_) {unit->setCurrentIndex(unit->findText(unit_));}
      void resize_(int rows, int cols) override { widget->resize_(rows,cols); }
      int rows() const override { return widget->rows(); }
      int cols() const override { return widget->cols(); }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class FromFileWidget : public VariableWidget {
    Q_OBJECT

    friend class FromFileProperty;

    public:
      FromFileWidget();
      QString getValue() const override;
      void setValue(const QString &str) override {}
      QString getFile() const {return relativeFilePath->text();}
      void setFile(const QString &str);
      QString getType() const override {return "File";}
      std::vector<std::vector<QString> > getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QLineEdit *relativeFilePath;
      QCheckBox *path;

    protected slots:
      void selectFile();
      void changePath(int i);
  };

  class BoolWidgetFactory : public WidgetFactory {
    public:
      BoolWidgetFactory(const QString &value_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      QString value;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class ScalarWidgetFactory : public WidgetFactory {
    public:
      ScalarWidgetFactory(const QString &value_, std::vector<QStringList> unit_=std::vector<QStringList>(2,noUnitUnits()), std::vector<int> defaultUnit_=std::vector<int>(2,0));
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      QString value;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class VecWidgetFactory : public WidgetFactory {
    public:
      VecWidgetFactory(int m, std::vector<QStringList> unit_=std::vector<QStringList>(3,noUnitUnits()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool transpose_=false, bool table_=false, bool eval_=true);
      VecWidgetFactory(std::vector<QString> x_, std::vector<QStringList> unit_=std::vector<QStringList>(3,noUnitUnits()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool transpose_=false, bool eval_=true);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> x;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
      bool transpose, table, eval;
  };

  class VecSizeVarWidgetFactory : public WidgetFactory {
    public:
      VecSizeVarWidgetFactory(int m_, int singleStep_=1, std::vector<QStringList> unit_=std::vector<QStringList>(3,noUnitUnits()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool transpose_=false, bool table_=false, bool eval_=true);
//      VecSizeVarWidgetFactory(const std::vector<QString> &x, const std::vector<QStringList> &unit=std::vector<QStringList>(3,noUnitUnits()), const std::vector<int> &defaultUnit=std::vector<int>(3,0), bool transpose=false);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      int m, singleStep;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
      bool transpose, table, eval;
  };

  class MatWidgetFactory : public WidgetFactory {
    public:
      MatWidgetFactory(int m, int n, std::vector<QStringList> unit_=std::vector<QStringList>(3,noUnitUnits()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool table_=false);
      MatWidgetFactory(std::vector<std::vector<QString> > A_, std::vector<QStringList> unit_=std::vector<QStringList>(3,noUnitUnits()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool table_=false);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<std::vector<QString> > A;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
      bool table;
  };

  class MatRowsVarWidgetFactory : public WidgetFactory {
    public:
      MatRowsVarWidgetFactory(int m_, int n_, std::vector<QStringList> unit_=std::vector<QStringList>(3,noUnitUnits()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool table_=false);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      int m, n;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
      bool table;
  };

  class MatColsVarWidgetFactory : public WidgetFactory {
    public:
      MatColsVarWidgetFactory(int m_, int n_, std::vector<QStringList> unit_=std::vector<QStringList>(3,noUnitUnits()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool table_=false);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      int m, n;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
      bool table;
  };

  class MatRowsColsVarWidgetFactory : public WidgetFactory {
    public:
      MatRowsColsVarWidgetFactory(int m=0, int n=0, bool table_=false);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<std::vector<QString> > A;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
      bool table;
  };

  class SqrMatSizeVarWidgetFactory : public WidgetFactory {
    public:
      SqrMatSizeVarWidgetFactory(int m_, std::vector<QStringList> unit_=std::vector<QStringList>(3,noUnitUnits()), std::vector<int> defaultUnit_=std::vector<int>(3,0));
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      int m;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class SymMatWidgetFactory : public WidgetFactory {
    public:
      SymMatWidgetFactory(std::vector<std::vector<QString> > A_, std::vector<QStringList> unit_=std::vector<QStringList>(3,noUnitUnits()), std::vector<int> defaultUnit_=std::vector<int>(3,0));
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<std::vector<QString> > A;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class SymMatSizeVarWidgetFactory : public WidgetFactory {
    public:
      SymMatSizeVarWidgetFactory(std::vector<std::vector<QString> > A_, std::vector<QStringList> unit_=std::vector<QStringList>(3,noUnitUnits()), std::vector<int> defaultUnit_=std::vector<int>(3,0));
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<std::vector<QString> > A;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class RotMatWidgetFactory : public WidgetFactory {
    public:
      RotMatWidgetFactory();
      RotMatWidgetFactory(std::vector<QString> name_, std::vector<QStringList> unit_, std::vector<int> defaultUnit_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

}

#endif
