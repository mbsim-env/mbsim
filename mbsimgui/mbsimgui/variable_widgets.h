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

namespace MBSimGUI {

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
      virtual QString getType() const = 0;
      virtual bool validate(const std::vector<std::vector<QString> > &A) const {return true;}
      virtual QWidget* getValidatedWidget() const {return 0;}
      virtual int rows() const { return 1; }
      virtual int cols() const { return 1; }
  };

  class BoolWidget : public VariableWidget {

    public:
      BoolWidget(const QString &b="0");
      QString getValue() const {return value->checkState()==Qt::Checked?"1":"0";}
      void setValue(const QString &str) {value->setCheckState((str=="0"||str=="false")?Qt::Unchecked:Qt::Checked);}
      virtual QString getType() const {return "Boolean";}
      bool validate(const std::vector<std::vector<QString> > &A) const;
      virtual QWidget* getValidatedWidget() const;

    protected:
      QCheckBox *value;
  };

  class ExpressionWidget : public VariableWidget {
    public:
      ExpressionWidget(const QString &str="");
      QString getValue() const { return value->toPlainText(); }
      void setValue(const QString &str) { value->setPlainText(str); }
      virtual QString getType() const {return "Editor";}
      virtual QWidget* getValidatedWidget() const;
      std::vector<std::vector<QString> > getMat() const;
      int rows() const { return getMat().size(); }
      int cols() const { return getMat().size()?getMat()[0].size():0; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    private:
      QPlainTextEdit *value;
  };

  class ScalarWidget : public VariableWidget {
    private:
      QLineEdit* box;
    public:
      ScalarWidget(const QString &d="1");
      void setReadOnly(bool flag) {box->setReadOnly(flag);}
      QString getValue() const {return box->text().isEmpty()?"0":box->text();}
      void setValue(const QString &str) {box->setText(str=="0"?"":str);}
      virtual QString getType() const {return "Scalar";}
      bool validate(const std::vector<std::vector<QString> > &A) const;
      virtual QWidget* getValidatedWidget() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
  };

  class BasicVecWidget : public VariableWidget {
    public:
      virtual std::vector<QString> getVec() const = 0;
      virtual void setVec(const std::vector<QString> &x) = 0;
      virtual QWidget* getValidatedWidget() const;
  };

  class VecWidget : public BasicVecWidget {
    private:
      std::vector<QLineEdit*> box;
      bool transpose;
    public:
      VecWidget(int size, bool transpose=false);
      VecWidget(const std::vector<QString> &x, bool transpose=false);
      void resize_(int size);
      void resize_(int rows, int cols) { resize_(rows); }
      std::vector<QString> getVec() const;
      void setVec(const std::vector<QString> &x);
      void setReadOnly(bool flag);
      QString getValue() const {return toQStr(getVec());}
      void setValue(const QString &str) {setVec(strToVec(str));}
      int size() const {return box.size();}
      int rows() const { return size(); }
      virtual QString getType() const {return "Vector";}
      bool validate(const std::vector<std::vector<QString> > &A) const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
  };

  class VecSizeVarWidget : public BasicVecWidget {

    Q_OBJECT

    private:
      VecWidget *widget;
      CustomSpinBox* sizeCombo;
      int minSize, maxSize;
    public:
      VecSizeVarWidget(int size, int minSize, int maxSize, bool transpose=false);
      std::vector<QString> getVec() const {return widget->getVec();}
      void setVec(const std::vector<QString> &x);
      void resize_(int size);
      int size() const {return sizeCombo->value();}
      int rows() const { return size(); }
      int cols() const { return 1; }
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

  class BasicMatWidget : public VariableWidget {
    public:
      virtual std::vector<std::vector<QString> > getMat() const = 0;
      virtual void setMat(const std::vector<std::vector<QString> > &A) = 0;
      virtual QWidget* getValidatedWidget() const;
  };

  class MatWidget : public BasicMatWidget {

    private:
      std::vector<std::vector<QLineEdit*> > box;
    public:
      MatWidget(int rows, int cols);
      MatWidget(const std::vector<std::vector<QString> > &A);
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

  class MatColsVarWidget : public BasicMatWidget {

    Q_OBJECT

    private:
      MatWidget *widget;
      QLabel *rowsLabel;
      CustomSpinBox* colsCombo;
      int minCols, maxCols;
    public:
      MatColsVarWidget(int rows, int cols, int minCols, int maxCols);
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
      CustomSpinBox *rowsCombo;
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
      CustomSpinBox *rowsCombo, *colsCombo;
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

  class SqrMatSizeVarWidget : public BasicMatWidget {

    Q_OBJECT

    private:
      MatWidget *widget;
      CustomSpinBox *sizeCombo;
      int minSize, maxSize;
    public:
      SqrMatSizeVarWidget(int size, int minSize, int maxSize);
      std::vector<std::vector<QString> > getMat() const {return widget->getMat();}
      void setMat(const std::vector<std::vector<QString> > &A);
      void resize_(int rows, int cols);
      int rows() const {return sizeCombo->value();}
      int cols() const {return cols();}
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

  class SymMatWidget : public BasicMatWidget {

    private:
      std::vector<std::vector<QLineEdit*> > box;
    public:
      SymMatWidget(int rows);
      SymMatWidget(const std::vector<std::vector<QString> > &A);
      void resize_(int rows);
      void resize_(int rows, int cols) { resize_(rows); }
      std::vector<std::vector<QString> > getMat() const;
      void setMat(const std::vector<std::vector<QString> > &A);
      void setReadOnly(bool flag);
      QString getValue() const {return toQStr(getMat());}
      void setValue(const QString &str) {setMat(strToMat(str));}
      int rows() const {return box.size();}
      int cols() const {return box[0].size();}
      virtual QString getType() const {return "Matrix";}
      bool validate(const std::vector<std::vector<QString> > &A) const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
  };

  class SymMatSizeVarWidget : public BasicMatWidget {

    Q_OBJECT

    private:
      SymMatWidget *widget;
      CustomSpinBox *sizeCombo;
      int minSize, maxSize;
    public:
      SymMatSizeVarWidget(int size, int minSize, int maxSize);
      std::vector<std::vector<QString> > getMat() const {return widget->getMat();}
      void setMat(const std::vector<std::vector<QString> > &A);
      void resize_(int rows, int cols);
      int rows() const {return sizeCombo->value();}
      int cols() const {return cols();}
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

  class CardanWidget : public VariableWidget {

    private:
      std::vector<QLineEdit*> box;
      QComboBox* unit;
    public:
      CardanWidget();
      std::vector<QString> getAngles() const;
      void setAngles(const std::vector<QString> &x);
      void setReadOnly(bool flag);
      QString getValue() const {return toQStr(getAngles());}
      void setValue(const QString &str) {setAngles(strToVec(str));}
      int size() const {return box.size();}
      virtual QString getType() const {return "Cardan";}
      bool validate(const std::vector<std::vector<QString> > &A) const;
      QString getUnit() const {return unit->currentText();}
      void setUnit(const QString &unit_) {unit->setCurrentIndex(unit->findText(unit_));}
      virtual QWidget* getValidatedWidget() const;
  };

  class AboutZWidget : public VariableWidget {

    private:
      QLineEdit* box;
      QComboBox* unit;
    public:
      AboutZWidget();
      QString getValue() const {return box->text().isEmpty()?"0":box->text();}
      void setValue(const QString &str) {box->setText(str=="0"?"":str);}
      virtual QString getType() const {return "AboutZ";}
      bool validate(const std::vector<std::vector<QString> > &A) const;
      QString getUnit() const {return unit->currentText();}
      void setUnit(const QString &unit_) {unit->setCurrentIndex(unit->findText(unit_));}
      virtual QWidget* getValidatedWidget() const;
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
      PhysicalVariableWidget(VariableWidget *widget, const QStringList &units=QStringList(), int defaultUnit=0);
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
      int rows() const { return widget->rows(); }
      int cols() const { return widget->cols(); }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
  };

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
      virtual QWidget* getValidatedWidget() const;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);

    protected:
      QLineEdit *relativeFilePath;

    protected slots:
      void selectFile();
  };

  class BoolWidgetFactory : public WidgetFactory {
    public:
      BoolWidgetFactory(const QString &value);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      QString value;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class ScalarWidgetFactory : public WidgetFactory {
    public:
      ScalarWidgetFactory(const QString &value, const std::vector<QStringList> &unit=std::vector<QStringList>(2,noUnitUnits()), const std::vector<int> &defaultUnit=std::vector<int>(2,0));
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      QString value;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class VecWidgetFactory : public WidgetFactory {
    public:
//      VecWidgetFactory(int m, bool transpose=false);
//      VecWidgetFactory(int m, const std::vector<QString> &name, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit, bool transpose=false);
      //VecWidgetFactory(int m, const std::vector<QStringList> &unit, bool transpose=false);
      VecWidgetFactory(int m, const std::vector<QStringList> &unit=std::vector<QStringList>(3,noUnitUnits()), const std::vector<int> &defaultUnit=std::vector<int>(3,0), bool transpose=false);
      VecWidgetFactory(const std::vector<QString> &x, const std::vector<QStringList> &unit=std::vector<QStringList>(3,noUnitUnits()), const std::vector<int> &defaultUnit=std::vector<int>(3,0), bool transpose=false);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> x;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
      bool transpose;
  };

  class VecSizeVarWidgetFactory : public WidgetFactory {
    public:
      VecSizeVarWidgetFactory(int m, bool transpose=false);
      VecSizeVarWidgetFactory(int m, const std::vector<QString> &name, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit, bool transpose=false);
      VecSizeVarWidgetFactory(int m, const std::vector<QStringList> &unit, bool transpose=false);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      int m;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
      bool transpose;
  };

  class MatWidgetFactory : public WidgetFactory {
    public:
      MatWidgetFactory();
      MatWidgetFactory(int m, int n, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
      MatWidgetFactory(const std::vector<std::vector<QString> > &A, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
      MatWidgetFactory(const std::vector<std::vector<QString> > &A, const std::vector<QString> &name, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<std::vector<QString> > A;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class MatRowsVarWidgetFactory : public WidgetFactory {
    public:
      MatRowsVarWidgetFactory();
      MatRowsVarWidgetFactory(int m, int n, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
      MatRowsVarWidgetFactory(int m, int n, const std::vector<QString> &name, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
//      std::vector<std::vector<QString> > A;
      int m, n;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class MatColsVarWidgetFactory : public WidgetFactory {
    public:
      MatColsVarWidgetFactory();
      MatColsVarWidgetFactory(int m, int n, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
      MatColsVarWidgetFactory(int m, int n, const std::vector<QString> &name, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
//      std::vector<std::vector<QString> > A;
      int m, n;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class MatRowsColsVarWidgetFactory : public WidgetFactory {
    public:
      MatRowsColsVarWidgetFactory(int m=0, int n=0);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<std::vector<QString> > A;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class SqrMatSizeVarWidgetFactory : public WidgetFactory {
    public:
      SqrMatSizeVarWidgetFactory();
      SqrMatSizeVarWidgetFactory(int m, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
      SqrMatSizeVarWidgetFactory(int m, const std::vector<QString> &name, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      int m;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class SymMatWidgetFactory : public WidgetFactory {
    public:
      SymMatWidgetFactory(const std::vector<std::vector<QString> > &A, const std::vector<QStringList> &unit=std::vector<QStringList>(3,noUnitUnits()), const std::vector<int> &defaultUnit=std::vector<int>(3,0));
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<std::vector<QString> > A;
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

  class SymMatSizeVarWidgetFactory : public WidgetFactory {
    public:
      SymMatSizeVarWidgetFactory();
      SymMatSizeVarWidgetFactory(const std::vector<std::vector<QString> > &A, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
      SymMatSizeVarWidgetFactory(const std::vector<std::vector<QString> > &A, const std::vector<QString> &name, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
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
      RotMatWidgetFactory(const std::vector<QString> &name, const std::vector<QStringList> &unit, const std::vector<int> &defaultUnit);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<QStringList> unit;
      std::vector<int> defaultUnit;
  };

}

#endif
