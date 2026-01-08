/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#ifndef _STRING_WIDGETS_H_
#define _STRING_WIDGETS_H_

#include "widget.h"
#include "utils.h"
#include "custom_widgets.h"
#include <QCheckBox>
#include <QPlainTextEdit>
#include <QLineEdit>

class QLabel;
class QTableWidget;

namespace MBSimGUI {

  class VariableWidget : public Widget {

    public:
      virtual void setReadOnly(bool flag) {}
      virtual QString getValue() const = 0;
      virtual void setValue(const QString &str) = 0;
      virtual void setDefaultValue(const QString &str) { }

      //! 0: floating point value (scalar, vector or matrix)
      //! 1: string value
      virtual int getVarType() const { return 0; }

      virtual bool validate(const std::vector<std::vector<QString>> &A) const { return true; }
      virtual int rows() const { return 1; }
      virtual int cols() const { return 1; }
      virtual std::vector<std::vector<QString>> getEvalMat() const;
  };

  class StringWidget : public VariableWidget {
    private:
      QLineEdit* box;
    public:
      StringWidget(const QString &d="", const QString &p="\"\"");
      void setReadOnly(bool flag) override {box->setReadOnly(flag);}
      QString getValue() const override { return box->text().isEmpty()?box->placeholderText():box->text(); }
      void setValue(const QString &str) override {box->setText(str);}
      int getVarType() const override { return 1; }
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      std::vector<std::vector<QString>> getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class BoolWidget : public VariableWidget {

    public:
      BoolWidget(const QString &b="0");
      QString getValue() const override;
      void setValue(const QString &str) override;
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      std::vector<std::vector<QString>> getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;

    protected:
      QCheckBox *value;
  };

  class ExpressionWidget : public VariableWidget {
    public:
      //! varType_: see VariableWidget::getVarType()
      ExpressionWidget(const QString &str="", int varType_=0);
      QString getValue() const override { return value->toPlainText(); }
      void setValue(const QString &str) override { value->setPlainText(str); }
      int getVarType() const override { return varType; }
      int rows() const override { return getEvalMat().size(); }
      int cols() const override { return !getEvalMat().empty()?getEvalMat()[0].size():0; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      int getStretchHint() const override { return 10; }

    private:
      QPlainTextEdit *value;
      int varType;
  };

  class ScalarWidget : public VariableWidget {
    private:
      QLineEdit* box;
      QString defaultValue;
    public:
      ScalarWidget(const QString &d="1", QString defaultValue_="0");
      void setReadOnly(bool flag) override {box->setReadOnly(flag);}
      QString getValue() const override { return box->text().isEmpty()?defaultValue:box->text(); }
      void setValue(const QString &str) override {box->setText(str);}
      void setDefaultValue(const QString &str) override { defaultValue = str; box->setPlaceholderText(defaultValue);}
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      std::vector<std::vector<QString>> getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class BasicVecWidget : public VariableWidget {
    public:
      virtual std::vector<QString> getVec() const = 0;
      virtual void setVec(const std::vector<QString> &x) = 0;
      std::vector<std::vector<QString>> getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class VecWidget : public BasicVecWidget {
    private:
      std::vector<QLineEdit*> box;
      bool transpose;
      QString defaultValue;
    public:
      VecWidget(int size, bool transpose_=false, QString defaultValue_="0");
      VecWidget(const std::vector<QString> &x, bool transpose_=false);
      void resize_(int size);
      void resize_(int rows, int cols) override { resize_(rows); }
      std::vector<QString> getVec() const override;
      void setVec(const std::vector<QString> &x) override;
      void setReadOnly(bool flag) override;
      QString getValue() const override { return toQStr(getVec()); }
      void setValue(const QString &str) override {setVec(strToVec(str));}
      int size() const { return box.size(); }
      int rows() const override { return size(); }
      bool validate(const std::vector<std::vector<QString>> &A) const override;
  };

  class VecSizeVarWidget : public BasicVecWidget {
    Q_OBJECT

    private:
      BasicVecWidget *widget;
      CustomSpinBox* sizeCombo;
      int minSize, maxSize;
    public:
      VecSizeVarWidget(int size, int minSize_, int maxSize_, int singleStep=1, bool transpose=false, bool table=false, QString defaultValue="0");
      std::vector<QString> getVec() const override { return widget->getVec(); }
      void setVec(const std::vector<QString> &x) override;
      void resize_(int size);
      void resize_(int rows, int cols) override { resize_(rows); }
      int size() const { return sizeCombo->value(); }
      int rows() const override { return size(); }
      int cols() const override { return 1; }
      QString getValue() const override { return toQStr(getVec()); }
      void setValue(const QString &str) override {setVec(strToVec(str));}
      void setReadOnly(bool flag) override {widget->setReadOnly(flag);}
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      void currentIndexChanged(int);
    Q_SIGNALS:
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
      QString getValue() const override { return toQStr(getVec()); }
      void setValue(const QString &str) override {setVec(strToVec(str));}
      int size() const;
      int rows() const override { return size(); }
      bool validate(const std::vector<std::vector<QString>> &A) const override;
  };

  class BasicMatWidget : public VariableWidget {
    public:
      virtual std::vector<std::vector<QString>> getMat() const = 0;
      virtual void setMat(const std::vector<std::vector<QString>> &A) = 0;
      std::vector<std::vector<QString>> getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class MatWidget : public BasicMatWidget {

    private:
      std::vector<std::vector<QLineEdit*>> box;
    public:
      MatWidget(int rows, int cols);
      MatWidget(const std::vector<std::vector<QString>> &A);
      void resize_(int rows, int cols) override;
      std::vector<std::vector<QString>> getMat() const override;
      void setMat(const std::vector<std::vector<QString>> &A) override;
      void setReadOnly(bool flag) override;
      QString getValue() const override { return toQStr(getMat()); }
      void setValue(const QString &str) override {setMat(strToMat(str));}
      int rows() const override { return box.size(); }
      int cols() const override { return box[0].size(); }
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      QLineEdit* getLineEdit(int i, int j) { return box[i][j]; }
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
      std::vector<std::vector<QString>> getMat() const override { return widget->getMat(); }
      void setMat(const std::vector<std::vector<QString>> &A) override;
      void resize_(int rows, int cols) override;
      int rows() const override { return widget->rows(); }
      int cols() const override { return colsCombo->value(); }
      QString getValue() const override { return toQStr(getMat()); }
      void setValue(const QString &str) override {setMat(strToMat(str));}
      void setReadOnly(bool flag) override {widget->setReadOnly(flag);}
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      void currentIndexChanged(int);
    Q_SIGNALS:
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
      std::vector<std::vector<QString>> getMat() const override  { return widget->getMat(); }
      void setMat(const std::vector<std::vector<QString>> &A) override;
      void resize_(int rows, int cols) override;
      int rows() const override { return rowsCombo->value(); }
      int cols() const override { return widget->cols(); }
      QString getValue() const override { return toQStr(getMat()); }
      void setValue(const QString &str) override {setMat(strToMat(str));}
      void setReadOnly(bool flag) override {widget->setReadOnly(flag);}
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      void currentIndexChanged(int);
    Q_SIGNALS:
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
      std::vector<std::vector<QString>> getMat() const override { return widget->getMat(); }
      void setMat(const std::vector<std::vector<QString>> &A) override;
      void resize_(int rows, int cols) override;
      int rows() const override { return rowsCombo->value(); }
      int cols() const override { return colsCombo->value(); }
      QString getValue() const override { return toQStr(getMat()); }
      void setValue(const QString &str) override {setMat(strToMat(str));}
      void setReadOnly(bool flag) override {widget->setReadOnly(flag);}
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      void currentRowIndexChanged(int);
      void currentColIndexChanged(int);
    Q_SIGNALS:
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
      std::vector<std::vector<QString>> getMat() const override { return widget->getMat(); }
      void setMat(const std::vector<std::vector<QString>> &A) override;
      void resize_(int rows, int cols) override;
      int rows() const override { return sizeCombo->value(); }
      int cols() const override { return rows(); }
      QString getValue() const override { return toQStr(getMat()); }
      void setValue(const QString &str) override {setMat(strToMat(str));}
      void setReadOnly(bool flag) override {widget->setReadOnly(flag);}
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      void currentIndexChanged(int);
    Q_SIGNALS:
      void sizeChanged(int);
  };

  class SymMatWidget : public BasicMatWidget {

    private:
      std::vector<std::vector<QLineEdit*>> box;
    public:
      SymMatWidget(int rows);
      SymMatWidget(const std::vector<std::vector<QString>> &A);
      void resize_(int rows);
      void resize_(int rows, int cols) override { resize_(rows); }
      std::vector<std::vector<QString>> getMat() const override;
      void setMat(const std::vector<std::vector<QString>> &A) override;
      void setReadOnly(bool flag) override;
      QString getValue() const override { return toQStr(getMat()); }
      void setValue(const QString &str) override {setMat(strToMat(str));}
      int rows() const override { return box.size(); }
      int cols() const override { return box[0].size(); }
      bool validate(const std::vector<std::vector<QString>> &A) const override;
  };

  class SymMatSizeVarWidget : public BasicMatWidget {
    Q_OBJECT

    private:
      SymMatWidget *widget;
      CustomSpinBox *sizeCombo;
      int minSize, maxSize;
    public:
      SymMatSizeVarWidget(int size, int minSize_, int maxSize_);
      std::vector<std::vector<QString>> getMat() const override { return widget->getMat(); }
      void setMat(const std::vector<std::vector<QString>> &A) override;
      void resize_(int rows, int cols) override;
      int rows() const override { return sizeCombo->value(); }
      int cols() const override { return rows(); }
      QString getValue() const override { return toQStr(getMat()); }
      void setValue(const QString &str) override {setMat(strToMat(str));}
      void setReadOnly(bool flag) override {widget->setReadOnly(flag);}
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      void currentIndexChanged(int);
    Q_SIGNALS:
      void sizeChanged(int);
  };

  class MatTableWidget: public BasicMatWidget {

    private:
      QTableWidget *table;
    public:
      MatTableWidget(int rows, int cols);
      MatTableWidget(const std::vector<std::vector<QString>> &A);
      void resize_(int rows, int cols) override;
      std::vector<std::vector<QString>> getMat() const override;
      void setMat(const std::vector<std::vector<QString>> &A) override;
      QString getValue() const override { return toQStr(getMat()); }
      void setValue(const QString &str) override {setMat(strToMat(str));}
      int rows() const override;
      int cols() const override;
      bool validate(const std::vector<std::vector<QString>> &A) const override;
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
      QString getValue() const override { return toQStr(getAngles()); }
      void setValue(const QString &str) override {setAngles(strToVec(str));}
      int size() const { return box.size(); }
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      QString getUnit() const { return unit->currentText(); }
      void setUnit(const QString &unit_) {unit->setCurrentIndex(unit->findText(unit_));}
      std::vector<std::vector<QString>> getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class AboutXWidget : public VariableWidget {

    private:
      QLineEdit* box;
      QComboBox* unit;
    public:
      AboutXWidget();
      QString getValue() const override { return box->text().isEmpty()?"0":box->text(); }
      void setValue(const QString &str) override {box->setText(str);}
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      QString getUnit() const { return unit->currentText(); }
      void setUnit(const QString &unit_) {unit->setCurrentIndex(unit->findText(unit_));}
      std::vector<std::vector<QString>> getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class AboutYWidget : public VariableWidget {

    private:
      QLineEdit* box;
      QComboBox* unit;
    public:
      AboutYWidget();
      QString getValue() const override { return box->text().isEmpty()?"0":box->text(); }
      void setValue(const QString &str) override {box->setText(str);}
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      QString getUnit() const { return unit->currentText(); }
      void setUnit(const QString &unit_) {unit->setCurrentIndex(unit->findText(unit_));}
      std::vector<std::vector<QString>> getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class AboutZWidget : public VariableWidget {

    private:
      QLineEdit* box;
      QComboBox* unit;
    public:
      AboutZWidget();
      QString getValue() const override { return box->text().isEmpty()?"0":box->text(); }
      void setValue(const QString &str) override {box->setText(str);}
      bool validate(const std::vector<std::vector<QString>> &A) const override;
      QString getUnit() const { return unit->currentText(); }
      void setUnit(const QString &unit_) {unit->setCurrentIndex(unit->findText(unit_));}
      std::vector<std::vector<QString>> getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class PhysicalVariableWidget : public VariableWidget {

    private:
      void openEvalDialog();
      VariableWidget *widget;
      QComboBox* unit;
      QStringList units;
      int defaultUnit;
    public:
      PhysicalVariableWidget(VariableWidget *widget_, const QStringList &units_=QStringList(), int defaultUnit_=0, bool eval=true);
      QString getValue() const override { return widget->getValue(); }
      void setValue(const QString &str) override { widget->setValue(str); }
      void setDefaultValue(const QString &str) override { widget->setDefaultValue(str); }
      void setReadOnly(bool flag) override { widget->setReadOnly(flag); }
      int getStretchHint() const override { return widget->getStretchHint(); }
      const QStringList& getUnitList() const { return units; }
      int getDefaultUnit() const { return defaultUnit; }
      bool validate(const std::vector<std::vector<QString>> &A) const override { return widget->validate(A); }
      QString getUnit() const { return unit->currentText(); }
      void setUnit(const QString &unit_) { unit->setCurrentIndex(unit->findText(unit_)); }
      void resize_(int rows, int cols) override { widget->resize_(rows,cols); }
      int rows() const override { return widget->rows(); }
      int cols() const override { return widget->cols(); }
      std::vector<std::vector<QString>> getEvalMat() const override { return widget->getEvalMat(); }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      VariableWidget* getWidgetVirtual() const override { return widget; }
  };

  class FromFileWidget : public VariableWidget {

    friend class FromFileProperty;

    public:
      FromFileWidget();
      QString getValue() const override;
      void setValue(const QString &str) override {}
      QString getFile() const { return relativeFilePath->text(); }
      void setFile(const QString &str);
      std::vector<std::vector<QString>> getEvalMat() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void selectFile();
      void changePath(int i);
      QLineEdit *relativeFilePath;
      QCheckBox *path;
  };

  class StringWidgetFactory : public WidgetFactory {
    public:
      StringWidgetFactory(const QString &value_, const QString &placeholderText_="\"\"");
      Widget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      QString value, placeholderText;
      std::vector<QString> name;
  };

  class BoolWidgetFactory : public WidgetFactory {
    public:
      BoolWidgetFactory(const QString &value_);
      Widget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      QString value;
      std::vector<QString> name;
  };

  class ScalarWidgetFactory : public WidgetFactory {
    public:
      ScalarWidgetFactory(const QString &value_, std::vector<QStringList> unit_=std::vector<QStringList>(2,QStringList()), std::vector<int> defaultUnit_=std::vector<int>(2,0), QString defaultValue_="0");
      Widget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      QString value;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
      QString defaultValue;
  };

  class VecWidgetFactory : public WidgetFactory {
    public:
      VecWidgetFactory(int m, std::vector<QStringList> unit_=std::vector<QStringList>(3,QStringList()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool transpose_=false, bool table_=false, bool eval_=true);
      VecWidgetFactory(std::vector<QString> x_, std::vector<QStringList> unit_=std::vector<QStringList>(3,QStringList()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool transpose_=false, bool eval_=true);
      Widget* createWidget(int i=0) override;
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
      VecSizeVarWidgetFactory(int m_, int mMin_=1, int mMax_=100, int singleStep_=1, std::vector<QStringList> unit_=std::vector<QStringList>(3,QStringList()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool transpose_=false, bool table_=false, bool eval_=true, QString defaultValue_="0");
//      VecSizeVarWidgetFactory(const std::vector<QString> &x, const std::vector<QStringList> &unit=std::vector<QStringList>(3,QStringList()), const std::vector<int> &defaultUnit=std::vector<int>(3,0), bool transpose=false);
      Widget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      int m, mMin, mMax, singleStep;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
      bool transpose, table, eval;
      QString defaultValue;
  };

  class MatWidgetFactory : public WidgetFactory {
    public:
      MatWidgetFactory(int m, int n, std::vector<QStringList> unit_=std::vector<QStringList>(3,QStringList()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool table_=false);
      MatWidgetFactory(std::vector<std::vector<QString>> A_, std::vector<QStringList> unit_=std::vector<QStringList>(3,QStringList()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool table_=false);
      Widget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<std::vector<QString>> A;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
      bool table;
  };

  class MatRowsVarWidgetFactory : public WidgetFactory {
    public:
      MatRowsVarWidgetFactory(int m_, int n_, std::vector<QStringList> unit_=std::vector<QStringList>(3,QStringList()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool table_=false);
      Widget* createWidget(int i=0) override;
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
      MatColsVarWidgetFactory(int m_, int n_, std::vector<QStringList> unit_=std::vector<QStringList>(3,QStringList()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool table_=false);
      Widget* createWidget(int i=0) override;
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
      MatRowsColsVarWidgetFactory(int m=0, int n=0, std::vector<QStringList> unit_=std::vector<QStringList>(3,QStringList()), std::vector<int> defaultUnit_=std::vector<int>(3,0), bool table_=false);
      Widget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<std::vector<QString>> A;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
      bool table;
  };

  class SqrMatSizeVarWidgetFactory : public WidgetFactory {
    public:
      SqrMatSizeVarWidgetFactory(int m_, std::vector<QStringList> unit_=std::vector<QStringList>(3,QStringList()), std::vector<int> defaultUnit_=std::vector<int>(3,0));
      Widget* createWidget(int i=0) override;
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
      SymMatWidgetFactory(std::vector<std::vector<QString>> A_, std::vector<QStringList> unit_=std::vector<QStringList>(3,QStringList()), std::vector<int> defaultUnit_=std::vector<int>(3,0));
      Widget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<std::vector<QString>> A;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class SymMatSizeVarWidgetFactory : public WidgetFactory {
    public:
      SymMatSizeVarWidgetFactory(std::vector<std::vector<QString>> A_, std::vector<QStringList> unit_=std::vector<QStringList>(3,QStringList()), std::vector<int> defaultUnit_=std::vector<int>(3,0));
      Widget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<std::vector<QString>> A;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class RotMatWidgetFactory : public WidgetFactory {
    public:
      RotMatWidgetFactory();
      RotMatWidgetFactory(std::vector<QString> name_, std::vector<QStringList> unit_, std::vector<int> defaultUnit_);
      Widget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

}

#endif
