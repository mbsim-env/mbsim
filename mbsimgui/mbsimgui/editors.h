#ifndef _EDITORS_H_
#define _EDITORS_H_

#include <QDialog>
#include <QAction>
#include <QGridLayout>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QScrollArea>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QTreeWidgetItem>
#include <QGroupBox>
#include <QSyntaxHighlighter>
#include "utils.h"
#include "mbsimguitinyxml/tinyxml-src/tinyxml.h"
#include "mbsimguitinyxml/tinyxml-src/tinynamespace.h"
#include <limits>

#define MBSIMNS_ "http://mbsim.berlios.de/MBSim"
#define MBSIMNS "{"MBSIMNS_"}"
#define PARAMNS_ "http://openmbv.berlios.de/MBXMLUtils/parameter"
#define PARAMNS "{"PARAMNS_"}"
#define PVNS_ "http://openmbv.berlios.de/MBXMLUtils/physicalvariable"
#define PVNS "{"PVNS_"}"

extern int digits;

template<class T> 
T max(T x1, T x2) {
  return x1>=x2?x1:x2;
}

class Editor;
class FrameBrowser;
class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;
class QPlainTextEdit;
class DMatWidget;
class Element;
class RigidBody;
class Frame;
class Joint;
class Parameter;
class QStackedLayout;

class OctaveHighlighter : public QSyntaxHighlighter {

  public:

    OctaveHighlighter(QTextDocument *parent);

  protected:

    void highlightBlock(const QString &text);
    std::vector<std::pair<QRegExp, QTextCharFormat> > rule;
};

class XMLWidget : public QWidget {
  public:
    virtual void initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual void initialize() {};
    virtual void update() {}
};

//class InputWidget : public QWidget {
//  public:
//    virtual bool initializeUsingXML(TiXmlElement *element) = 0;
//    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
//    virtual std::string getValue() const = 0;
//    virtual void setValue(const std::string &str) = 0;
//};

class QElementItem : public QTreeWidgetItem {
  private:
    Element* element;
  public:
    QElementItem(Element *element_) : element(element_) {}
    Element* getElement() const {return element;}
};

class IntEdit : public QLineEdit {
  Q_OBJECT

  public:
    IntEdit(QWidget * parent = 0) : QLineEdit(parent) {
      validator = new QIntValidator();
      setValidator(validator);
      setValue(0);
      QObject::connect(this, SIGNAL(textChanged(const QString&)), this, SLOT(sendSignal(const QString&)));
      QObject::connect(this, SIGNAL(textEdited(const QString&)), this, SLOT(sendSignal2(const QString&)));
    }
    double value() const {return text().toInt();}

  public slots:
    //void setValue(double d) { setText(QString::number(d,'g',std::numeric_limits<double>::digits10)+suffix); }
    void setValue(int i) { setText(QString::number(i)); }
    void sendSignal(const QString& str) { emit valueChanged(str.toDouble()); }
    void sendSignal2(const QString& str) { emit valueChanged2(str.toDouble()); }

  signals:
    void valueChanged(double);
    void valueChanged2(double);
  private:
    QIntValidator *validator;
};

class MyDoubleEdit : public QLineEdit {
  Q_OBJECT

  public:
    MyDoubleEdit(QWidget * parent = 0) : QLineEdit(parent) {
      validator = new QDoubleValidator();
      setValidator(validator);
      validator->setDecimals(std::numeric_limits<double>::digits10);
      setValue(0);
      QObject::connect(this, SIGNAL(textChanged(const QString&)), this, SLOT(sendSignal(const QString&)));
      QObject::connect(this, SIGNAL(textEdited(const QString&)), this, SLOT(sendSignal2(const QString&)));
    }
    //double value() const {QString buf = text(); buf.chop(suffix.size()); return buf.toDouble();}
    double value() const {return text().toDouble();}
    void setDecimals(int decimals) {validator->setDecimals(decimals);}
    void setRange(int min, int max) {};
    void setSingleStep(int step) {};
    void setSuffix(const QString &suffix_) {} //{suffix = suffix_;}

  public slots:
    //void setValue(double d) { setText(QString::number(d,'g',std::numeric_limits<double>::digits10)+suffix); }
    void setValue(double d) { setText(QString::number(d,'g',validator->decimals())); }
    void sendSignal(const QString& str) { emit valueChanged(str.toDouble()); }
    void sendSignal2(const QString& str) { emit valueChanged2(str.toDouble()); }

  signals:
    void valueChanged(double);
    void valueChanged2(double);
  private:
    QDoubleValidator *validator;
    //QString suffix;
};

class MyDoubleSpinBox : public QDoubleSpinBox {
  public:
    MyDoubleSpinBox(QWidget * parent = 0) : QDoubleSpinBox(parent) { 
        //cachedSizeHint = QDoubleSpinBox::sizeHint();
        QDoubleSpinBox::setDecimals(std::numeric_limits<double>::max_exponent);
    }
    QString textFromValue(double value) const {
     //   return QString::number(value, 'g', std::numeric_limits<double>::digits10);
//return locale().toString(value, 'f', std::numeric_limits<double>::digits10);
//      QDoubleSpinBox::textFromValue(value);
//    //Q_D(const QDoubleSpinBox);
//      std::cout << QString::number(value).toStdString() << std::endl;
//      std::cout << locale().toString(value, 'f', 5).toStdString() << std::endl;
//    QString str = QString::number(value);
//    if (qAbs(value) >= 1000.0) {
//        str.remove(locale().groupSeparator());
//    }
//    return str;
     return QString::number(value);
    }
 //   double valueFromText ( const QString & text ) const {
 //     return text.toDouble();
 //   }
     
//    QSize sizeHint() const {
//        return cachedSizeHint;
//    }
};

//typedef QDoubleSpinBox DoubleEdit;
typedef MyDoubleEdit DoubleEdit;
//typedef MyDoubleSpinBox DoubleEdit;

class Function1 : public QWidget {
  Q_OBJECT
  public:
    Function1() {}
    Function1(const QString& ext_) : ext(ext_) {}
    virtual ~Function1() {}
    virtual void initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *parent) {
      TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
      parent->LinkEndChild(ele0);
      return ele0;
    }
    virtual QString getType() const { return "Function1_"+ext; }
    virtual QString getExt() const { return ext; }
  public slots:
    virtual void resize(int m, int n) {}
  protected:
    QString ext;
};

class Function2 : public QWidget {
  Q_OBJECT
  public:
    Function2() {}
    Function2(const QString& ext_) : ext(ext_) {}
    virtual ~Function2() {}
    virtual void initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *parent) {
      TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
      parent->LinkEndChild(ele0);
      return ele0;
    }
    virtual QString getType() const { return "Function2_"+ext; }
    virtual QString getExt() const { return ext; }
  public slots:
    virtual void resize(int m, int n) {}
  protected:
    QString ext;
};

class DifferentiableFunction1 : public Function1 {
  public:
    DifferentiableFunction1() : Function1(), order(0) {}
    DifferentiableFunction1(const QString &ext) : Function1(ext), order(0) {}
    //virtual ~DifferentiableFunction1() { delete derivatives[0]; derivatives.erase(derivatives.begin()); }
    const Function1& getDerivative(int degree) const { return *(derivatives[degree]); }
    Function1& getDerivative(int degree) { return *(derivatives[degree]); }
    void addDerivative(Function1 *diff) { derivatives.push_back(diff); }
    void setDerivative(Function1 *diff,size_t degree) { derivatives.resize(max(derivatives.size(),degree+1)); derivatives[degree]=diff; }

    void setOrderOfDerivative(int i) { order=i; }

    virtual void initializeUsingXML(TiXmlElement *element) {
      Function1::initializeUsingXML(element);
      TiXmlElement * e;
      e=element->FirstChildElement(MBSIMNS"orderOfDerivative");
      if (e)
        setOrderOfDerivative(atoi(e->GetText()));
    }
    TiXmlElement* writeXMLFile(TiXmlNode *parent) {
      TiXmlElement *ele0 = Function1::writeXMLFile(parent);
      addElementText(ele0,MBSIMNS"orderOfDerivative",order);
      return ele0;
    }
    QString getType() const { return "DifferentiableFunction1"; }

  protected:
    std::vector<Function1*> derivatives;
    int order;
};

class ConstantFunction1 : public Function1 {
  public:
    ConstantFunction1(DMatWidget* ret, const QString &ext);
    virtual DMatWidget* getMatWidget() { return c; }
    void initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
    inline QString getType() const { return QString("ConstantFunction1_")+ext; }
    void resize(int m, int n);
  protected:
    DMatWidget *c;
};

class SinusFunction1 : public DifferentiableFunction1 {
  public:
    SinusFunction1(DMatWidget *amplitude, DMatWidget *frequency, DMatWidget *phase, DMatWidget *offset);
    void initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
    inline QString getType() const { return QString("SinusFunction1_VS"); }
    void resize(int m, int n);

 //   class ZerothDerivative : public Function1 {
 //      public:
 //       ZerothDerivative(SinusFunction1 *sin) : Function1(), parent(sin) {}
 //       Vector<Col,double> operator()(const double& x, const void * =NULL);
 //     private:
 //       SinusFunction1 *parent;
 //   };

 //   class FirstDerivative : public Function1 {
 //      public:
 //       FirstDerivative(SinusFunction1 *sin) : Function1(), parent(sin) {}
 //       Vector<Col,double> operator()(const double& x, const void * =NULL);
 //     private:
 //       SinusFunction1 *parent;
 //   };
 //   
 //   class SecondDerivative : public Function1 {
 //      public:
 //       SecondDerivative(SinusFunction1 *sin) : Function1(), parent(sin) {}
 //       Vector<Col,double> operator()(const double& x, const void * =NULL);
 //     private:
 //       SinusFunction1 *parent;
 //   };
  protected:
    int ySize;
    DMatWidget *amplitude, *frequency, *phase, *offset;
  private:
};

class LinearSpringDamperForce : public Function2 {
  public:
    LinearSpringDamperForce();
    void initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
    inline QString getType() const { return QString("LinearSpringDamperForce")+ext; }
  protected:
    DoubleEdit *c, *d, *l0;
};

class LinearRegularizedBilateralConstraint: public Function1 {
  public:
    LinearRegularizedBilateralConstraint(); 

    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
    virtual QString getType() const { return "LinearRegularizedBilateralConstraint"; }

  private:
    DoubleEdit *c, *d;
};


class DMatWidget : public QWidget {

  private:
    std::vector<std::vector<QLineEdit*> > box;
  public:
    DMatWidget(int rows, int cols, QWidget *parent = 0);
    void resize(int rows, int cols);
    void init();
    std::vector<std::vector<double> >getMat() const;
    void setMat(const std::vector<std::vector<double> > &A);
    void setReadOnly(bool flag);
    int rows() const {return box.size();}
    int cols() const {return box[0].size();}
};

class Mat3VWidget : public QWidget {

  Q_OBJECT

  private:
    DMatWidget *widget;
    QComboBox* colsCombo;
    int minCols, maxCols;
  public:
    Mat3VWidget(int cols, int minCols, int maxCols, QWidget *parent = 0);
    std::vector<std::vector<double> > getMat() const {return widget->getMat();}
    void setMat(const std::vector<std::vector<double> > &A) {
      colsCombo->setCurrentIndex(colsCombo->findText(QString::number(A[0].size())));
      widget->setMat(A);
    }
    QComboBox* getComboBox() {return colsCombo;}
    int rows() const {return 3;}
    int cols() const {return colsCombo->currentText().toInt();}

  public slots:
    void resize(const QString &text) {widget->resize(3,text.toInt());}//widget->init();}

};

// TODO Prüfen ob überflüssig
class StringWidget : public QWidget {

  public:
    virtual void setReadOnly(bool flag) {}
    virtual bool validate(const std::string &str) {return true;}
    virtual std::string getValue() const = 0;
    virtual void setValue(const std::string &str) = 0;
    virtual bool initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual StringWidget* cloneStringWidget() {return 0;}
};

class OctaveExpressionWidget : public StringWidget {
  public:
    OctaveExpressionWidget();
    std::string getValue() const { return value->toPlainText().toStdString(); }
    void setValue(const std::string &str) { value->setPlainText(str.c_str()); }
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  private:
    QPlainTextEdit *value;
};


class SScalarWidget : public StringWidget {
  Q_OBJECT

  private:
    QLineEdit* box;
  public:
    SScalarWidget(const std::string &d="1");
    void setReadOnly(bool flag) {box->setReadOnly(flag);}
    std::string getValue() const {return box->text().toStdString();}
    void setValue(const std::string &str) {box->setText(str.c_str());}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SScalarWidget;}
  //signals:
    //void valueChanged(const QString& d);
};

class SVecWidget : public StringWidget {

  private:
    std::vector<QLineEdit*> box;
    bool transpose;
  public:
    SVecWidget(int size, bool transpose=false);
    SVecWidget(const std::vector<std::string> &x, bool transpose=false);
    void resize(int size);
    std::vector<std::string> getVec() const;
    void setVec(const std::vector<std::string> &x);
    void setReadOnly(bool flag);
    std::string getValue() const {return toStr(getVec());}
    void setValue(const std::string &str) {setVec(strToSVec(str));}
    int size() const {return box.size();}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SVecWidget(size());}
};

class SMatWidget : public StringWidget {

  private:
    std::vector<std::vector<QLineEdit*> > box;
  public:
    SMatWidget(int rows, int cols);
    SMatWidget(const std::vector<std::vector<std::string> > &A);
    void resize(int rows, int cols);
    std::vector<std::vector<std::string> > getMat() const;
    void setMat(const std::vector<std::vector<std::string> > &A);
    void setReadOnly(bool flag);
    std::string getValue() const {return toStr(getMat());}
    void setValue(const std::string &str) {setMat(strToSMat(str));}
    int rows() const {return box.size();}
    int cols() const {return box[0].size();}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SMatWidget(rows(),cols());}
};

class SSymMatWidget : public StringWidget {

  private:
    std::vector<std::vector<QLineEdit*> > box;
  public:
    SSymMatWidget(int rows);
    SSymMatWidget(const std::vector<std::vector<std::string> > &A);
    void resize(int rows);
    std::vector<std::vector<std::string> > getMat() const;
    void setMat(const std::vector<std::vector<std::string> > &A);
    void setReadOnly(bool flag);
    std::string getValue() const {return toStr(getMat());}
    void setValue(const std::string &str) {setMat(strToSMat(str));}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SSymMatWidget(rows());}
    int rows() const {return box.size();}
    int cols() const {return box[0].size();}
};

class SVecVarWidget : public StringWidget {

  Q_OBJECT

  private:
    SVecWidget *widget;
    QComboBox* sizeCombo;
    int minSize, maxSize;
  public:
    SVecVarWidget(int size, int minSize, int maxSize);
    std::vector<std::string> getVec() const {return widget->getVec();}
    void setVec(const std::vector<std::string> &x) {
      sizeCombo->setCurrentIndex(sizeCombo->findText(QString::number(x.size())));
      widget->setVec(x);
    }
    void resize(int size) {widget->resize(size);}
    int size() const {return sizeCombo->currentText().toInt();}
    std::string getValue() const {return toStr(getVec());}
    void setValue(const std::string &str) {setVec(strToSVec(str));}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SVecWidget(size());}

  public slots:
    void resize(const QString &size) {widget->resize(size.toInt());}
  signals:
    void currentIndexChanged(int);

};

class SMatColsVarWidget : public StringWidget {

  Q_OBJECT

  private:
    SMatWidget *widget;
    QComboBox* colsCombo;
    int minCols, maxCols;
  public:
    SMatColsVarWidget(int rows, int cols, int minCols, int maxCols);
    std::vector<std::vector<std::string> > getMat() const {return widget->getMat();}
    void setMat(const std::vector<std::vector<std::string> > &A) {
      colsCombo->setCurrentIndex(colsCombo->findText(QString::number(A[0].size())));
      widget->setMat(A);
    }
    void resize(int rows, int cols) {widget->resize(rows,cols);}
    int rows() const {return widget->rows();}
    int cols() const {return colsCombo->currentText().toInt();}
    std::string getValue() const {return toStr(getMat());}
    void setValue(const std::string &str) {setMat(strToSMat(str));}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SMatWidget(rows(),cols());}

  public slots:
    void resize(const QString &cols) {widget->resize(widget->rows(),cols.toInt());}
  signals:
    void currentIndexChanged(int);

};

class SCardanWidget : public StringWidget {

  private:
    std::vector<QLineEdit*> box;
    bool transpose;
  public:
    SCardanWidget(bool transpose=false);
    SCardanWidget(const std::vector<std::string> &x, bool transpose=false);
    std::vector<std::string> getCardan() const;
    void setCardan(const std::vector<std::string> &x);
    void setReadOnly(bool flag);
    std::string getValue() const {return toStr(getCardan());}
    void setValue(const std::string &str) {setCardan(strToSVec(str));}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return new SCardanWidget;}
};

class PhysicalStringWidget : public StringWidget {

  Q_OBJECT

  private:
    StringWidget *widget;
    QComboBox* unit;
    std::string xmlName;
    QStringList units;
    int defaultUnit;
  public:
    PhysicalStringWidget(StringWidget *widget, const std::string &xmlname,  const QStringList &units, int defaultUnit);
    std::string getValue() const {return widget->getValue();}
    void setValue(const std::string &str) {widget->setValue(str);}
    void setReadOnly(bool flag) {widget->setReadOnly(flag);}
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual StringWidget* cloneStringWidget() {return widget->cloneStringWidget();}
    virtual StringWidget* getWidget() {return widget;}
    const std::string& getXmlName() const {return xmlName;}
    const QStringList& getUnitList() const {return units;}
    int getDefaultUnit() const {return defaultUnit;}
};

class PropertyDialog : public QScrollArea {
  Q_OBJECT

  friend class TransRotEditor;
  public:
    PropertyDialog(QObject *obj);
    ~PropertyDialog();
    void setParentObject(QObject *obj);
    void addToTab(const QString &name, QWidget* widget) {layout[name]->addWidget(widget);}
    void addToTab2(const QString &name, QWidget* widget) {layout2[name]->addWidget(widget);}
    void addTab(const QString &name);
    void updateHeader();
    QObject* getParentObject() { return parentObject; }
    void addEditor(Editor *child); 
    void addStretch() {
      for ( std::map<QString,QVBoxLayout*>::iterator it=layout.begin() ; it != layout.end(); it++ )
        (*it).second->addStretch(1);
      for ( std::map<QString,QVBoxLayout*>::iterator it=layout2.begin() ; it != layout2.end(); it++ )
        (*it).second->addStretch(1);
    }
    void update();
    void initialize();
  protected:
    QObject* parentObject;
    std::map<QString,QVBoxLayout*> layout, layout2;
    QVBoxLayout *mainLayout;
    std::vector<Editor*> editor;
    QTabWidget *tabWidget;
  protected slots:
};

class RigidBodyBrowser : public QDialog {
  Q_OBJECT

  public:
    RigidBodyBrowser(QTreeWidget* tree, RigidBody* selection, QWidget *obj);
    ~RigidBodyBrowser() {}
    QTreeWidget* getRigidBodyList() const {return rigidBodyList;}
    void update(RigidBody *rigidBody);
  protected:
    QPushButton *okButton;
    QTreeWidget *rigidBodyList;
    RigidBody *selection;
    QElementItem *savedItem;
    QTreeWidget* tree;
    void mbs2RigidBodyTree(Element* item, QTreeWidgetItem* parentItem);
  protected slots:
    void checkForRigidBody(QTreeWidgetItem* item_,int);
};

class FrameBrowser : public QDialog {
  Q_OBJECT

  public:
    FrameBrowser(QTreeWidget* tree, Frame* selection, QWidget *obj);
    ~FrameBrowser() {}
    QTreeWidget* getFrameList() const {return frameList;}
    void update(Frame *frame);
  protected:
    QPushButton *okButton;
    QTreeWidget *frameList;
    Frame *selection;
    QElementItem *savedItem;
    QTreeWidget* tree;
    void mbs2FrameTree(Element* item, QTreeWidgetItem* parentItem);
  protected slots:
    void checkForFrame(QTreeWidgetItem* item_,int);
};

class Editor : public QWidget {
  public:
    Editor(PropertyDialog *parent_, const QIcon &icon, const std::string &name);
    virtual void update() {}
    virtual void initialize() {}
  protected:
    PropertyDialog *dialog;
};

class BoolEditor : public Editor {

  public:
    BoolEditor(PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab="General", bool val=false);

    void setValue(bool b) {value->setCheckState(b?Qt::Checked:Qt::Unchecked);}
    int getValue() const {return value->checkState()==Qt::Checked?true:false;}

  protected:
    QCheckBox *value;
};

class IntEditor : public Editor {

  public:
    IntEditor(PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab="General", int val=1.);

    void setValue(int i) {value->setValue(i);}
    int getValue() const {return value->value();}

  protected:
    IntEdit *value;
};

class DoubleEditor : public Editor {
  Q_OBJECT

  public:
    DoubleEditor(PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab="General", double val=1., double singleStep=1., const QString &suffix="");

    void setValue(double d) {value->setValue(d);}
    double getValue() const {return value->value();}

  protected:
    DoubleEdit *value;
  signals:
    void valueChanged(double d);
};

class MatEditor : public Editor {
  Q_OBJECT

  public:
    MatEditor(PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab);

  void setSize(int rows, int cols) { A->resize(rows,cols); }
  int getRows() const { return A->rows(); }
  std::vector<std::vector<double> > getMat() const {return A->getMat();}
  void setMat(const std::vector<std::vector<double> > &A_) {A->setMat(A_);}

  protected:
    DMatWidget *A;
};


class NameEditor : public Editor {
  Q_OBJECT

  public:
    NameEditor(Element* ele, PropertyDialog *parent_, const QIcon &icon, const std::string &name, bool renaming=true);

    QString getName() const {return ename->text();}
    void setName(const QString &name) {ename->setText(name);}

  protected:
    QGroupBox *groupBox;
    QLineEdit *ename;
    Element* element;

  protected slots:
    void rename();
};

class LocalFrameOfReferenceWidget : public XMLWidget {
  Q_OBJECT

  public:
    LocalFrameOfReferenceWidget(const std::string &xmlName, Element* element, Frame* omitFrame=0);

    void update();
    Frame* getFrame() {return selectedFrame;}
    void setFrame(Frame* frame_);
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    QComboBox *frame;
    Element* element;
    Frame *selectedFrame, *omitFrame;
    std::string xmlName;

  protected slots:
    void setFrame(const QString &str);
};

class FrameOfReferenceWidget : public XMLWidget {
  Q_OBJECT

  public:
    FrameOfReferenceWidget(const std::string &xmlName, Element* element, Frame* selectedFrame);

    void initialize();
    void update();
    Frame* getFrame() {return selectedFrame;}
    void setFrame(Frame* frame_);
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void setSavedFrameOfReference(const QString &str) {saved_frameOfReference = str;}
    const QString& getSavedFrameOfReference() const {return saved_frameOfReference;}

  protected:
    QLineEdit *frame;
    Element* element;
    FrameBrowser *frameBrowser;
    Frame *selectedFrame;
    std::string xmlName;
    QString saved_frameOfReference;

  public slots:
    void setFrame();
};

class EvalDialog : public QDialog {
  Q_OBJECT
  public:
    EvalDialog(StringWidget *var);
    //void setMat(const std::vector<std::vector<double> > &A) {mat->setMat(A);}
    //std::vector<std::vector<double> > getMat() const {return mat->getMat();}
    void setValue(const std::string &str) {var->setValue(str);}
    std::string getValue() const {return var->getValue();}
    void setButtonDisabled(bool flag) {button->setDisabled(flag);}
  protected:
    StringWidget *var;
    QPushButton *button;
  signals:
    void clicked(bool);
};

//class XMLWidget : public QWidget {
//  public:
//    virtual void initializeUsingXML(TiXmlElement *element) = 0;
//    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
//};

class ExtPhysicalVarWidget : public XMLWidget {
  Q_OBJECT

  public:
    ExtPhysicalVarWidget(std::vector<PhysicalStringWidget*> inputWidget);

    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    PhysicalStringWidget* getPhysicalStringWidget(int i) {return inputWidget[i];}
    PhysicalStringWidget* getCurrentPhysicalStringWidget() {return inputWidget[inputCombo->currentIndex()];}
    virtual std::string getValue() const;

  protected:
    std::vector<PhysicalStringWidget*> inputWidget;
    QComboBox *inputCombo;
    EvalDialog *evalDialog;
    int evalInput;
  protected slots:
    void openEvalDialog();
    void updateInput();
  signals:
    void inputDialogChanged(int);
};

class TranslationWidget : public QWidget {

  public:
    TranslationWidget(QWidget *parent = 0) : QWidget(parent) {}
    virtual void initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual int getSize() const = 0;
   protected:
};

class LinearTranslation : public TranslationWidget {
  Q_OBJECT

  public:
    LinearTranslation(QWidget *parent = 0);
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const;
  protected:
    ExtPhysicalVarWidget *mat;
  protected slots:
    void checkInputSchema();
  signals:
    void currentIndexChanged(int);
};

class TranslationEditor : public Editor {
  Q_OBJECT

  public:
    /*! Constructor. */
    TranslationEditor(PropertyDialog *parent_, const QIcon &icon, const std::string &name);

    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const { return comboBox->currentIndex()>0?translation->getSize():0; }
    int getTranslation() {return comboBox->currentIndex();}

  protected slots:
    void defineTranslation(int);

  protected:
    QGroupBox *groupBox;
    QVBoxLayout *layout;
    QComboBox *comboBox;
    TranslationWidget *translation;
  signals:
    void translationChanged();
};

class RotationWidget : public QWidget {

  public:
    RotationWidget(QWidget *parent = 0) {}
    virtual void initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual int getSize() const = 0;
};

class RotationAboutXAxis : public RotationWidget {

  public:
    RotationAboutXAxis(QWidget *parent = 0) {}
    virtual void initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
};

class RotationAboutYAxis : public RotationWidget {

  public:
    RotationAboutYAxis(QWidget *parent = 0) {}
    virtual void initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
};

class RotationAboutZAxis : public RotationWidget {

  public:
    RotationAboutZAxis(QWidget *parent = 0) {}
    virtual void initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
};

class RotationAboutFixedAxis : public RotationWidget {

  public:
    RotationAboutFixedAxis(QWidget *parent = 0);
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    //std::vector<std::vector<double> > getAxisOfRotation() {return vec->getMat();}
    //void setAxisOfRotation(const std::vector<std::vector<double> > &x) {vec->setMat(x);}
    virtual int getSize() const {return 1;}
   protected:
    ExtPhysicalVarWidget *vec;
};

class RotationAboutAxesXY : public RotationWidget {

  public:
    RotationAboutAxesXY(QWidget *parent = 0) {}
    virtual void initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 2;}
};

class CardanAngles : public RotationWidget {

  public:
    CardanAngles(QWidget *parent = 0) {}
    virtual void initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 3;}
};

class RotationEditor : public Editor {
  Q_OBJECT

  public:
    /*! Constructor. */
    RotationEditor(PropertyDialog *parent_, const QIcon &icon, const std::string &name);

    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const { return comboBox->currentIndex()>0?rotation->getSize():0; }
    int getRotation() {return comboBox->currentIndex();}

  protected slots:
   void defineRotation(int);

  protected:
    QGroupBox *groupBox;
    QVBoxLayout *layout;
    QComboBox *comboBox;
    RotationWidget *rotation;
  signals:
    void rotationChanged();
};

class InitialGeneralizedCoordinatsEditor : public Editor {
  Q_OBJECT

  public:
    InitialGeneralizedCoordinatsEditor(PropertyDialog *parent_, const QIcon &icon, const std::string &name);

  void setq0Size(int size) { q0->resize(size,1); }
  void setu0Size(int size) { u0->resize(size,1); }
  int getq0Size() const { return q0->rows(); }
  int getu0Size() const { return u0->rows(); }
  std::vector<std::vector<double> > getq0() const {return q0->getMat();}
  std::vector<std::vector<double> > getu0() const {return u0->getMat();}
  void setq0(const std::vector<std::vector<double> > &x) {q0->setMat(x);}
  void setu0(const std::vector<std::vector<double> > &x) {u0->setMat(x);}

  protected:
    QGroupBox *groupBox;
    DMatWidget *q0, *u0;
};


class EnvironmentEditor : public Editor {
  Q_OBJECT

  public:
    EnvironmentEditor(PropertyDialog *parent_, const QIcon &icon, const std::string &name);

    void setAccelerationOfGravity(const std::vector<std::vector<double> > &g) {vec->setMat(g);}
    std::vector<std::vector<double> > getAccelerationOfGravity() {return vec->getMat();}
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
  protected:
    QGroupBox *groupBox;
    QVBoxLayout *layout;
    DMatWidget *vec;
};

class Vec3Editor : public Editor {
  Q_OBJECT

  public:
    Vec3Editor(PropertyDialog *parent_, const QIcon &icon, const std::string &name);

    std::vector<std::vector<double> >getVec() {return vec->getMat();}
    void setVec(const std::vector<std::vector<double> > &x) {vec->setMat(x);}

  protected:
    QGroupBox *groupBox;
    DMatWidget *vec;
};

class FramePositionWidget : public XMLWidget {

  public:
    FramePositionWidget(Frame *frame);

    void update() {refFrame->update();}
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    Frame *getFrame() {return frame;}

  protected:
    Frame *frame;
    ExtPhysicalVarWidget *position, *orientation;
    LocalFrameOfReferenceWidget *refFrame;
};

class FramePositionsWidget : public XMLWidget {

  public:
    FramePositionsWidget(Element *element);

    void update();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    Element *element;
    QStackedLayout *layout; 
    QListWidget *frameList; 
};

class OMBVBodyWidget : public QWidget {
  Q_OBJECT

  public:
    OMBVBodyWidget(RigidBody *body, QWidget *parent = 0);
    //virtual void update() {ref->update();}
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const { return "OMBVBody"; }
    //Frame* getFrame() const {return ref->getFrame();}
    //void setFrame(Frame *frame) {ref->setFrame(frame);}
  protected:
    RigidBody *body;
    QGridLayout *layout;
    ExtPhysicalVarWidget *trans, *rot, *color, *scale;
};

class CuboidWidget : public OMBVBodyWidget {
  Q_OBJECT

  public:
    CuboidWidget(RigidBody *body, QWidget *parent = 0);
    void initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Cuboid"; }
  protected:
    ExtPhysicalVarWidget *length;
};

class SphereWidget : public OMBVBodyWidget {
  Q_OBJECT

  public:
    SphereWidget(RigidBody *body, QWidget *parent = 0);
    void initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Sphere"; }
  protected:
    ExtPhysicalVarWidget *radius;
};

class FrustumWidget : public OMBVBodyWidget {
  Q_OBJECT

  public:
    FrustumWidget(RigidBody *body, QWidget *parent = 0);
    void initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Frustum"; }
  protected:
    ExtPhysicalVarWidget *top, *base, *height, *innerBase, *innerTop;
};

class OMBVEditor : public Editor {
  Q_OBJECT
  public:

    OMBVEditor(RigidBody* body, PropertyDialog *parent_, const QIcon &icon, const std::string &name);

    virtual void update() {ref->update();}
    int getOpenMBVBody() {return comboBox->currentIndex();}
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    protected slots:
      void ombvSelection(int index);

  protected:
    QGroupBox *groupBox;
    QVBoxLayout *layout;
    QComboBox *comboBox;
    RigidBody *body;
    OMBVBodyWidget *ombv;
    LocalFrameOfReferenceWidget *ref;
};

class FrameVisuEditor : public Editor {
  Q_OBJECT

  public:
    FrameVisuEditor(Frame* frame, PropertyDialog *parent_, const QIcon &icon, const std::string &name);

    bool openMBVFrame() const {return visu->checkState()==Qt::Checked;}
    double getSize() const {return size->value();}
    double getOffset() const {return offset->value();}
    void setOpenMBVFrame(bool b) {visu->setCheckState(b?Qt::Checked:Qt::Unchecked);}
    void setSize(double d) {size->setValue(d);}
    void setOffset(double d) {offset->setValue(d);}

  protected:
    QGroupBox *groupBox;
    QCheckBox *visu;
    Frame* frame;
    DoubleEdit *size, *offset;
};

class ConnectWidget : public XMLWidget {

  public:
    ConnectWidget(int n, Element* element);

    void initialize();
    void update();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    std::vector<FrameOfReferenceWidget*> widget;
    Element* element;
};

class GeneralizedForceLawWidget : public QWidget {

  public:
    GeneralizedForceLawWidget(QWidget *parent = 0) : forceFunc(0) {}
    virtual void initializeUsingXML(TiXmlElement *element) {};
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "GeneralizedForceLaw"; }
   protected:
    Function1 *forceFunc;
};

class BilateralConstraint : public GeneralizedForceLawWidget {

  public:
    BilateralConstraint(QWidget *parent = 0) : GeneralizedForceLawWidget(parent) {}
    virtual QString getType() const { return "BilateralConstraint"; }
   protected:
};

class RegularizedBilateralConstraint : public GeneralizedForceLawWidget {
  Q_OBJECT

  public:
    RegularizedBilateralConstraint(QWidget *parent = 0); 
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual QString getType() const { return "RegularizedBilateralConstraint"; }
  protected:
    QVBoxLayout *layout;
    QComboBox *funcList;
  protected slots:
    void defineFunction(int);
};

class GeneralizedImpactLawWidget : public QWidget {

  public:
    GeneralizedImpactLawWidget(QWidget *parent = 0) {}
    virtual void initializeUsingXML(TiXmlElement *element) {};
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "GeneralizedImpactLaw"; }
   protected:
};

class BilateralImpact : public GeneralizedImpactLawWidget {

  public:
    BilateralImpact(QWidget *parent = 0) : GeneralizedImpactLawWidget(parent) {}
    virtual QString getType() const { return "BilateralImpact"; }
   protected:
};

class GeneralizedForceLawEditor : public Editor {
  Q_OBJECT

  public:
    GeneralizedForceLawEditor(PropertyDialog *parent_, const QIcon &icon, bool force);

    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getForceLaw() {return comboBox->currentIndex();}
    int getSize() const { return mat->cols(); }

  protected slots:
    void defineForceLaw(int);

  protected:
    QGroupBox *groupBox;
    QVBoxLayout *layout;
    QComboBox *comboBox;
    GeneralizedForceLawWidget *generalizedForceLaw;
    GeneralizedImpactLawWidget *generalizedImpactLaw;
    Mat3VWidget *mat;
    bool force;
};

class ForceLawEditor : public Editor {
  Q_OBJECT

  public:
    ForceLawEditor(PropertyDialog *parent_, const QIcon &icon, bool force);

    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    //int getForceLaw() {return comboBox->currentIndex();}
    //int getSize() const { return mat->cols(); }

  protected slots:
    void defineForceLaw(int);
    void resize(int);

  protected:
    QGroupBox *groupBox;
    QVBoxLayout *layout;
    QComboBox *comboBox;
    Function1 *forceLaw;
    PhysicalStringWidget *mat;
    ExtPhysicalVarWidget *widget;
    bool force;
};

class ForceLawEditor2 : public Editor {
  Q_OBJECT

  public:
    ForceLawEditor2(Element *element, PropertyDialog *parent_, const QIcon &icon);

    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getForceLaw() {return comboBox->currentIndex();}
    void initialize();
    FrameOfReferenceWidget* getFrameOfReference() {return refFrame;}

  protected slots:
    void defineForceDir(bool);
    void defineForceLaw(int);
    void resize(int);

  protected:
    QVBoxLayout *layout;
    QPushButton *forceDirButton;
    QComboBox *comboBox;
    FrameOfReferenceWidget* refFrame;
    Element *element;
    Function2 *forceLaw;
    DMatWidget *mat;
    QString saved_frameOfReference;
};

class GeneralizedForceDirectionEditor : public Editor {

  public:
    GeneralizedForceDirectionEditor(PropertyDialog *parent_, const QIcon &icon, bool force);

    //virtual void initializeUsingXML(TiXmlElement *element);
    //virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    std::vector<std::vector<double> > getForceDir() {return mat->getMat();}
    void setForceDir(const std::vector<std::vector<double> > &dir) {mat->setMat(dir);}
    int getSize() const { return mat->cols(); }

  protected:
    Mat3VWidget *mat;
    bool force;
};

class RigidBodyOfReferenceWidget : public QWidget {
  Q_OBJECT

  public:
    RigidBodyOfReferenceWidget(Element* element);

    void update();
    RigidBody* getBody() {return selectedBody;}
    void setBody(RigidBody* body_);

  protected:
    QLineEdit* body;
    Element* element;
    RigidBodyBrowser* bodyBrowser;
    RigidBody* selectedBody;

  public slots:
    void setBody();

  signals:
    void bodyChanged();
};

class RigidBodyOfReferenceEditor : public Editor {

  public:
    RigidBodyOfReferenceEditor(Element* element, PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab);

    void update() {refBody->update();}
    RigidBody* getBody() {return refBody->getBody();}
    void setBody(RigidBody* body) {refBody->setBody(body);}

  protected:
    RigidBodyOfReferenceWidget *refBody;
};


class DependenciesEditor : public Editor {
  Q_OBJECT

  public:
    DependenciesEditor(Element* element, PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab);

    void update() {
      for(unsigned int i=0; i<refBody.size(); i++)
        refBody[i]->update();
    }
    RigidBody* getBody(int i) {return refBody[i]->getBody();}
    void setBody(int i, RigidBody* body) {refBody[i]->setBody(body);}
    void setBody(int i) {refBody[i]->setBody();}
    void addBody(int i, RigidBody* body_);
    int getSize() const {return refBody.size();}
    void setBodies(std::vector<RigidBody*> rigidBodies);

  protected:
    QGridLayout *layout;
    Element* element;
    std::vector<RigidBody*> selectedBody;
    std::vector<RigidBodyOfReferenceWidget*> refBody;
    std::vector<QPushButton*> button;
    std::vector<QLabel*> label;

  protected slots:
    void addDependency();
    void removeDependency();
    void updateGeneralizedCoordinatesOfBodies();

  signals:
      void bodyChanged();
};

class DoubleParameterWidget : public QWidget {

  public:
    DoubleParameterWidget(QWidget *parent = 0);
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
  protected:
    QLineEdit *name;
    DoubleEdit *value;
};

class DoubleParameterEditor : public Editor {

  public:
    DoubleParameterEditor(PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab="Parameters");
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
  protected:
    DoubleParameterWidget *parameter;
};

class ParameterEditor : public Editor {
  Q_OBJECT

  public:
    ParameterEditor(PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab="Parameters");

    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void writeXMLFile(const QString &name);
  protected:
    QGridLayout *layout;
    std::vector<DoubleParameterWidget*> parameter;
  protected slots:
    void addParameter();
};

class ParameterNameEditor : public Editor {
  Q_OBJECT

  public:
    ParameterNameEditor(Parameter* ele, PropertyDialog *parent_, const QIcon &icon, const std::string &name, bool renaming=true);

    QString getName() const {return ename->text();}
    void setName(const QString &name) {ename->setText(name);}

  protected:
    QLineEdit *ename;
    Parameter* parameter;

  protected slots:
    void rename();
};

class FileEditor : public Editor {
  Q_OBJECT

  public:
    FileEditor(PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab);
    QString getFile() const {return fileName->text();}
    void setFile(const QString &file) {fileName->setText(file);}

  protected:
    QLineEdit *fileName;

  protected slots:
    void selectFile();

};

class ParameterValueEditor : public Editor {
  Q_OBJECT

  public:
    ParameterValueEditor(PhysicalStringWidget *var, PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab);

//    virtual void initializeUsingXML(TiXmlElement *element);
//    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
   //StringWidget* getStringWidget() {return var;}
   ExtPhysicalVarWidget* getExtPhysicalWidget() {return widget;}
   virtual std::string getValue() const { return widget->getValue(); }

  protected:
    ExtPhysicalVarWidget *widget;
  protected slots:
    void parameterChanged();
  signals:
    void parameterChanged(const QString &str);
};

class XMLEditor : public Editor {

  public:
    XMLEditor(PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab, XMLWidget *d);

    virtual void initializeUsingXML(TiXmlElement *element) {widget->initializeUsingXML(element);}
    virtual TiXmlElement* writeXMLFile(TiXmlElement *element) {return widget->writeXMLFile(element);}
    XMLWidget* getXMLWidget() {return widget;}
    virtual void initialize() {widget->initialize();}
    virtual void update() {widget->update();}

  protected:
    XMLWidget *widget;
};

class GeneralizedCoordinatesEditor : public Editor {
  Q_OBJECT

  public:
    GeneralizedCoordinatesEditor(PropertyDialog *parent_, const QIcon &icon, const QString &name, const QString &tab, const std::string &xmlName);

    virtual void initializeUsingXML(TiXmlElement *element) {widget->initializeUsingXML(element);}
    virtual TiXmlElement* writeXMLFile(TiXmlElement *element) {return widget->writeXMLFile(element);}
    ExtPhysicalVarWidget* getExtPhysicalWidget() {return widget;}
    virtual void update() {widget->update();}

  protected:
    ExtPhysicalVarWidget *widget;
    QPushButton *buttonDisable, *buttonResize;
  protected slots:
    void updateButtons(int i);
  signals:
    void resizeGeneralizedCoordinates();
    void disableGeneralizedCoordinates();
};

#endif
