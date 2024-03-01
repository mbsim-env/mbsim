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

#include <config.h>
#include "basic_widgets.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "rigid_body.h"
#include "signal_.h"
#include "constraint.h"
#include "dialogs.h"
#include "utils.h"
#include "variable_widgets.h"
#include "mainwindow.h"
#include "project.h"
#include "xml_highlighter.h"
#include <QLabel>
#include <QColorDialog>
#include <boost/lexical_cast.hpp>
#include <utility>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMLSSerializer.hpp>
#include <xercesc/dom/DOMLSInput.hpp>
#include <xercesc/dom/DOMComment.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  LocalFrameComboBox::LocalFrameComboBox(Element *element_, QWidget *parent) : CustomComboBox(parent), element(element_) {
    connect(this,QOverload<const QString&>::of(&QComboBox::
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
      textHighlighted
#else
      highlighted
#endif
      ),this,&LocalFrameComboBox::highlightObject);
  }

  void LocalFrameComboBox::showPopup() {
    CustomComboBox::showPopup();
    highlightObject(currentText());
  }

  void LocalFrameComboBox::hidePopup() {
    CustomComboBox::hidePopup();
    if(not oldID.empty()) {
      mw->highlightObject(oldID);
      oldID="";
    }
  }

  void LocalFrameComboBox::highlightObject(const QString &frame) {
    if(oldID=="") 
      oldID = mw->getHighlightedObject();
    Frame *selection = element->getFrame(frame.mid(6, frame.length()-7));
    if(selection)
      mw->highlightObject(selection->getID());
  }

  ParentFrameComboBox::ParentFrameComboBox(Element *element_, QWidget *parent) : CustomComboBox(parent), element(element_) {
    connect(this,QOverload<const QString&>::of(&QComboBox::
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
      textHighlighted
#else
      highlighted
#endif
      ),this,&ParentFrameComboBox::highlightObject);
  }

  void ParentFrameComboBox::showPopup() {
    CustomComboBox::showPopup();
    highlightObject(currentText());
  }

  void ParentFrameComboBox::hidePopup() {
    CustomComboBox::hidePopup();
    if(not oldID.empty()) {
      mw->highlightObject(oldID);
      oldID="";
    }
  }

  void ParentFrameComboBox::highlightObject(const QString &frame) {
    if(oldID=="") 
      oldID = mw->getHighlightedObject();
    Frame *selection = element->getParent()->getFrame(frame.mid(9, frame.length()-10));
    if(selection)
      mw->highlightObject(selection->getID());
  }

  LocalFrameOfReferenceWidget::LocalFrameOfReferenceWidget(Element *element_, Frame* omitFrame_) : element(element_), selectedFrame(nullptr), omitFrame(omitFrame_) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    frame = new LocalFrameComboBox(element);
    frame->setEditable(true);
    layout->addWidget(frame);
    selectedFrame = element->getFrame(0);
    connect(frame,QOverload<int>::of(&QComboBox::currentIndexChanged),this,&LocalFrameOfReferenceWidget::setFrame);
    updateWidget();
  }

  void LocalFrameOfReferenceWidget::updateWidget() {
    frame->blockSignals(true);
    frame->clear();
    int oldIndex = 0;
    QString oldText = frame->currentText();
    for(int i=0, k=0; i<element->getNumberOfFrames(); i++) {
      if(omitFrame!=element->getFrame(i)) {
        frame->addItem("Frame["+element->getFrame(i)->getName()+"]");
        if(element->getFrame(i) == selectedFrame)
          oldIndex = k;
        k++;
      }
    }
    if(selectedFrame)
      frame->setCurrentIndex(oldIndex);
    else
      frame->setEditText(oldText);
    frame->blockSignals(false);
  }

  void LocalFrameOfReferenceWidget::setFrame(int index) {
    QString str=frame->itemText(index);
    selectedFrame = element->getFrame(str.mid(6, str.length()-7));
    frame->setEditText(str);
  }

  QString LocalFrameOfReferenceWidget::getFrame() const {
    return frame->currentText();
  }

  DOMElement* LocalFrameOfReferenceWidget::initializeUsingXML(DOMElement *xmlElement) {
    QString str=QString::fromStdString(E(xmlElement)->getAttribute("ref"));
    selectedFrame = element->getFrame(str.mid(6, str.length()-7));
    frame->setEditText(str);
    return xmlElement;
  }

  DOMElement* LocalFrameOfReferenceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(static_cast<DOMElement*>(parent))->setAttribute("ref", getFrame().toStdString());
    return nullptr;
  }

  ParentFrameOfReferenceWidget::ParentFrameOfReferenceWidget(Element *element_, Frame* omitFrame_) : element(element_), selectedFrame(nullptr), omitFrame(omitFrame_) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    frame = new ParentFrameComboBox(element);
    frame->setEditable(true);
    layout->addWidget(frame);
    selectedFrame = element->getParent()->getFrame(0);
    connect(frame,QOverload<int>::of(&QComboBox::currentIndexChanged),this,&ParentFrameOfReferenceWidget::setFrame);
    updateWidget();
  }

  void ParentFrameOfReferenceWidget::updateWidget() {
    frame->blockSignals(true);
    frame->clear();
    int oldIndex = 0;
    QString oldText = frame->currentText();
    for(int i=0, k=0; i<element->getParent()->getNumberOfFrames(); i++) {
      if(omitFrame!=element->getParent()->getFrame(i)) {
        frame->addItem("../Frame["+element->getParent()->getFrame(i)->getName()+"]");
        if(element->getParent()->getFrame(i) == selectedFrame)
          oldIndex = k;
        k++;
      }
    }
    if(selectedFrame)
      frame->setCurrentIndex(oldIndex);
    else
      frame->setEditText(oldText);
    frame->blockSignals(false);
  }

  void ParentFrameOfReferenceWidget::setFrame(int index) {
    QString str=frame->itemText(index);
    selectedFrame = element->getParent()->getFrame(str.mid(9, str.length()-10));
    frame->setEditText(str);
  }

  QString ParentFrameOfReferenceWidget::getFrame() const {
    return frame->currentText();
  }

  DOMElement* ParentFrameOfReferenceWidget::initializeUsingXML(DOMElement *xmlElemewnt) {
    QString str=QString::fromStdString(E(xmlElemewnt)->getAttribute("ref"));
    selectedFrame = element->getParent()->getFrame(str.mid(9, str.length()-10));
    frame->setEditText(str);
    return xmlElemewnt;
  }

  DOMElement* ParentFrameOfReferenceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(static_cast<DOMElement*>(parent))->setAttribute("ref", getFrame().toStdString());
    return nullptr;
  }

  BasicElementOfReferenceWidget::BasicElementOfReferenceWidget(Element *element_, Element* selectedElement, BasicElementBrowser *eleBrowser_, bool addRatio) : ratio(nullptr), element(element_), eleBrowser(eleBrowser_) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    ele = new QLineEdit;
    if(selectedElement)
      setElement(selectedElement->getXMLPath(element,true));
    layout->addWidget(ele);

    QPushButton *button = new QPushButton(tr("Browse"));
    connect(eleBrowser,&BasicElementBrowser::accepted,this,QOverload<>::of(&BasicElementOfReferenceWidget::setElement));
    connect(eleBrowser,&BasicElementBrowser::accepted,this,&BasicElementOfReferenceWidget::widgetChanged);
    connect(button,&QPushButton::clicked,this,&BasicElementOfReferenceWidget::showBrowser);
    layout->addWidget(button);

    if(addRatio) {
      QLabel *label = new QLabel("Ratio");
      layout->addWidget(label);
      ratio = new QLineEdit;
      ratio->setPlaceholderText("0");
      layout->addWidget(ratio);
    }
  }

  void BasicElementOfReferenceWidget::setElement() {
    Element *selectedElement = eleBrowser->getSelection();
    ele->setText(selectedElement?selectedElement->getXMLPath(element,true):"");
  }

  void BasicElementOfReferenceWidget::showBrowser() {
    eleBrowser->setSelection(findElement(ele->text()));
    eleBrowser->show();
  }

  DOMElement* BasicElementOfReferenceWidget::initializeUsingXML(DOMElement *element) {
    setElement(QString::fromStdString(E(element)->getAttribute("ref")));
    if(ratio) setRatio(QString::fromStdString(E(element)->getAttribute("ratio")));
    return element;
  }

  DOMElement* BasicElementOfReferenceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(static_cast<DOMElement*>(parent))->setAttribute("ref", getElement().toStdString());
    if(ratio) E(static_cast<DOMElement*>(parent))->setAttribute("ratio", getRatio().toStdString());
    return nullptr;
  }

  FileWidget::FileWidget(const QString &file, const QString &description_, const QString &extensions_, int mode_, bool quote_, bool absPath, QFileDialog::Options options_) : description(description_), extensions(extensions_), mode(mode_), quote(quote_), options(options_) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    filePath = new QLineEdit;
    layout->addWidget(filePath);
    QPushButton *button = new QPushButton("Browse");
    layout->addWidget(button);
    connect(button,&QPushButton::clicked,this,&FileWidget::selectFile);
    connect(filePath,&QLineEdit::textChanged,this,&FileWidget::valueChanged);
    path = new QCheckBox;
    setFile(file);
    if(absPath) {
      layout->addWidget(new QLabel("Absolute"));
      layout->addWidget(path);
      connect(path,&QCheckBox::stateChanged,this,&FileWidget::changePath);
    }
  }

  void FileWidget::setFile(const QString &str) {
    filePath->setText(str);
    QString file = quote?str.mid(1,str.length()-2):str;
    path->setChecked(QDir::isAbsolutePath(file));
  }

  void FileWidget::selectFile() {
    QString file = getFile(true);
    if(mode==0) 
      file = QFileDialog::getOpenFileName(this, description, path->isChecked()?file:mw->getProjectDir().absoluteFilePath(file), extensions, nullptr, options);
    else if(mode==1)
      file = QFileDialog::getSaveFileName(this, description, path->isChecked()?file:mw->getProjectDir().absoluteFilePath(file), extensions, nullptr, options);
    else
      file = QFileDialog::getExistingDirectory(this, description, path->isChecked()?file:mw->getProjectDir().absoluteFilePath(file));
    if(not file.isEmpty()) {
      if(path->isChecked())
        filePath->setText(quote?("\""+mw->getProjectDir().absoluteFilePath(file)+"\""):mw->getProjectDir().absoluteFilePath(file));
      else
        filePath->setText(quote?("\""+mw->getProjectDir().relativeFilePath(file)+"\""):mw->getProjectDir().relativeFilePath(file));
    }
  }

  DOMElement* FileWidget::initializeUsingXML(DOMElement *parent) {
    DOMText *text = E(parent)->getFirstTextChild();
    if(text) {
      setFile(QString::fromStdString(X()%text->getData()));
      return parent;
    }
    return nullptr;
  }

  DOMElement* FileWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    auto *ele0 = static_cast<DOMElement*>(parent);
    DOMText *text = doc->createTextNode(X()%getFile().toStdString());
    ele0->insertBefore(text, nullptr);
    return nullptr;
  }

  void FileWidget::changePath(int i) {
    QString file = getFile(true);
    if(i)
      filePath->setText(quote?("\""+mw->getProjectDir().absoluteFilePath(file)+"\""):mw->getProjectDir().absoluteFilePath(file));
    else
      filePath->setText(quote?("\""+mw->getProjectDir().relativeFilePath(file)+"\""):mw->getProjectDir().relativeFilePath(file));
  }

  SpinBoxWidget::SpinBoxWidget(int val, int min, int max) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    value = new CustomSpinBox;
    value->setValue(val);
    value->setMinimum(min);
    value->setMaximum(max);
    layout->addWidget(value);
    connect(value,QOverload<int>::of(&CustomSpinBox::valueChanged),this,&SpinBoxWidget::valueChanged);
  }

  DOMElement* SpinBoxWidget::initializeUsingXML(DOMElement *element) {
    DOMText *text = E(element)->getFirstTextChild();
    if(text) {
      value->blockSignals(true);
      setValue(boost::lexical_cast<int>(X()%text->getData()));
      value->blockSignals(false);
      return element;
    }
    return nullptr;
  }

  DOMElement* SpinBoxWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text= doc->createTextNode(X()%toStr(getValue()));
    parent->insertBefore(text, nullptr);
    return nullptr;
  }

  DOMElement* BasicTextWidget::initializeUsingXML(DOMElement *element) {
    DOMText *text_ = E(element)->getFirstTextChild();
    if(text_) {
      setText(QString::fromStdString(X()%text_->getData()));
      return element;
    }
    return nullptr;
  }

  DOMElement* BasicTextWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text_ = doc->createTextNode(X()%getText().toStdString());
    parent->insertBefore(text_, nullptr);
    return nullptr;
  }

  TextWidget::TextWidget(const QString &text_, bool readOnly) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    text = new QLineEdit;
    text->setText(text_);
    text->setReadOnly(readOnly);
    layout->addWidget(text);
    connect(text,&QLineEdit::textEdited,this,&Widget::widgetChanged);
  }

  DOMElement* TextWidget::initializeUsingXML(DOMElement *element) {
    text->blockSignals(true);
    DOMElement *ele = BasicTextWidget::initializeUsingXML(element);
    text->blockSignals(false);
    return ele;
  }

  TextChoiceWidget::TextChoiceWidget(const vector<QString> &list, int num, bool editable) {
    text = new CustomComboBox;
    text->setEditable(editable);
    for(const auto & i : list)
      text->addItem(i);
    text->setCurrentIndex(num);
    auto* layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    layout->addWidget(text);
    connect(text,&CustomComboBox::currentTextChanged,this,&Widget::widgetChanged);
  }

  void TextChoiceWidget::setStringList(const vector<QString> &list) {
    QString cur = text->currentText();
    text->clear();
    for(const auto & i : list)
      text->addItem(i);
    if(cur.isEmpty())
      text->setCurrentIndex(0);
    else
      text->setCurrentText(cur);
  }

  void TextChoiceWidget::setCurrentIndex(int num) {
    text->setCurrentIndex(num);
  }

  int TextChoiceWidget::getCurrentIndex() const {
    return text->currentIndex();
  }

  DOMElement* TextChoiceWidget::initializeUsingXML(DOMElement *element) {
    text->blockSignals(true);
    DOMElement *ele = BasicTextWidget::initializeUsingXML(element);
    text->blockSignals(false);
    return ele;
  }

  BasicConnectElementsWidget::BasicConnectElementsWidget(const vector<BasicElementOfReferenceWidget*> widget_, const vector<QString> &name) : widget(widget_) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    for(size_t i=0; i<widget.size(); i++) {
      QString subname = name[i];
      if(widget.size()>1 and name[1]==name[0])
        subname += QString(" ")+QString::number(i+1);
      QWidget *subwidget = new ExtWidget(subname,widget[i]);
      layout->addWidget(subwidget);
    }
  }

  void BasicConnectElementsWidget::updateWidget() {
    for(auto & i : widget)
      i->updateWidget();
  }

  DOMElement* BasicConnectElementsWidget::initializeUsingXML(DOMElement *element) {
    for(unsigned int i=0; i<widget.size(); i++) {
      string xmlName = "ref";
      if(widget.size()>1)
        xmlName += toStr(int(i+1));
      if(E(element)->hasAttribute(xmlName))
        widget[i]->setElement(QString::fromStdString(E(element)->getAttribute(xmlName)));
      else if(def.isEmpty())
        return nullptr;
    }
    return element;
  }

  DOMElement* BasicConnectElementsWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    for(unsigned int i=0; i<widget.size(); i++) {
      string xmlName = "ref";
      if(widget.size()>1)
        xmlName += toStr(int(i+1));
      if(i>0 or widget[i]->getElement()!=def)
        E(static_cast<DOMElement*>(parent))->setAttribute(xmlName, widget[i]->getElement().toStdString());
    }
    return nullptr;
  }

  ColorWidget::ColorWidget(const vector<QString> &c) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    color = new ExtWidget("HSV",new ChoiceWidget(new VecWidgetFactory(c),QBoxLayout::RightToLeft,5),false,false,"");
    layout->addWidget(color);

    button = new QPushButton(tr("Select"));
    connect(button,&QPushButton::clicked,this,&ColorWidget::setColor);
    layout->addWidget(button);
  }

  void ColorWidget::setColor() { 
    QString val = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(color->getWidget())->getWidget())->getValue();
    vector<QString> vec = strToVec(val);
    QColor col;
    if(vec.size()==3)
      col = QColorDialog::getColor(QColor::fromHsvF(vec[0].toDouble(),vec[1].toDouble(),vec[2].toDouble()),this);
    else
      col = QColorDialog::getColor(Qt::blue,this);
    if(col.isValid()) {
      QString str = "[" + QString::number(col.hueF()) + ";" + QString::number(col.saturationF()) + ";" + QString::number(col.valueF()) + "]";
      static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(color->getWidget())->getWidget())->setValue(str);
    }
  }

  DOMElement* ColorWidget::initializeUsingXML(DOMElement *parent) {
    color->initializeUsingXML(parent);
    return parent;
  }

  DOMElement* ColorWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    color->writeXMLFile(parent);
    return nullptr;
  }
 
  PlotFeatureWidget::PlotFeatureWidget(const FQN &specialType_) : specialType(specialType_) {
    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);

    dialog = new PlotFeatureDialog(specialType,this);
    dialog->setModal(true);

    tree = new QTreeWidget;
    tree->setMinimumSize(300,500);
    QStringList labels;
    labels << "Type" << "Value" << "Status" << "Namespace";
    tree->setHeaderLabels(labels);
    layout->addWidget(tree,0,0,4,1);
    tree->setColumnWidth(0,200);
    tree->setColumnWidth(1,150);
    tree->setColumnWidth(2,50);
    tree->setColumnWidth(3,250);

    connect(dialog, &PlotFeatureDialog::accepted, this, &PlotFeatureWidget::updateTreeItem);

    QPushButton *add = new QPushButton("Add");
    connect(add,&QPushButton::clicked,this,QOverload<>::of(&PlotFeatureWidget::addFeature));
    layout->addWidget(add,0,1);

    QPushButton *update = new QPushButton("Edit");
    connect(update,&QPushButton::clicked,this,&PlotFeatureWidget::editFeature);
    connect(tree,&QTreeWidget::itemDoubleClicked,this,&PlotFeatureWidget::editFeature);
    layout->addWidget(update,1,1);

    QPushButton *remove = new QPushButton("Remove");
    connect(remove,&QPushButton::clicked,this,&PlotFeatureWidget::removeFeature);
    layout->addWidget(remove,2,1);

    layout->setColumnStretch(0,10);
  }

  void PlotFeatureWidget::updateTreeItem() {
    QTreeWidgetItem *item = tree->currentItem();
    if(item) {
      item->setText(0, dialog->getType());
      item->setText(1, dialog->getValue());
      item->setText(2, dialog->getStatus());
      item->setText(3, dialog->getNamespace());
    }
  }

  void PlotFeatureWidget::addFeature() {
    auto *item = new QTreeWidgetItem;
    tree->addTopLevelItem(item);
    tree->setCurrentItem(item);
    dialog->show();
  }

  void PlotFeatureWidget::editFeature() {
    QTreeWidgetItem *item = tree->currentItem();
    if(item) {
      dialog->setType(item->text(0));
      dialog->setValue(item->text(1));
      dialog->setStatus(item->text(2));
      dialog->setNamespace(item->text(3));
      dialog->show();
    }
  }

  void PlotFeatureWidget::removeFeature() {
    tree->takeTopLevelItem(tree->indexOfTopLevelItem(tree->currentItem()));
  }

  DOMElement* PlotFeatureWidget::initializeUsingXML(DOMElement *parent) {
    DOMElement *e=parent->getFirstElementChild();
    while(e && (E(e)->getTagName()==MBSIM%"plotFeature" ||
                E(e)->getTagName()==MBSIM%"plotFeatureForChildren" ||
                E(e)->getTagName()==MBSIM%"plotFeatureRecursive")) {
      auto *item = new QTreeWidgetItem;
      item->setText(0, QString::fromStdString(E(e)->getTagName().second));
      item->setText(1, QString::fromStdString(E(e)->getAttributeQName("value").second));
      item->setText(2, QString::fromStdString(X()%E(e)->getFirstTextChild()->getData()));
      item->setText(3, QString::fromStdString(E(e)->getAttributeQName("value").first));
      tree->addTopLevelItem(item);
      e=e->getNextElementSibling();
    }
    return e;
  }

  DOMElement* PlotFeatureWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    for(size_t i=0; i<tree->topLevelItemCount(); i++) {
      DOMElement *ele = D(doc)->createElement(MBSIM%tree->topLevelItem(i)->text(0).toStdString());
      E(ele)->setAttribute("value",NamespaceURI(tree->topLevelItem(i)->text(3).toStdString())%tree->topLevelItem(i)->text(1).toStdString());
      ele->insertBefore(doc->createTextNode(X()%tree->topLevelItem(i)->text(2).toStdString()), nullptr);
      parent->insertBefore(ele, ref);
    }
    return nullptr;
  }

  DOMElement* PlotFeatureWidget::initializeUsingXML2(DOMElement *parent) {
    DOMElement *e=E(parent)->getFirstElementChildNamed(specialType);
    while(e && E(e)->getTagName()==specialType) {
      auto *item = new QTreeWidgetItem;
      item->setText(0, QString::fromStdString(E(e)->getTagName().second));
      item->setText(1, QString::fromStdString(E(e)->getAttributeQName("value").second));
      item->setText(2, QString::fromStdString(X()%E(e)->getFirstTextChild()->getData()));
      item->setText(3, QString::fromStdString(E(e)->getAttributeQName("value").first));
      tree->addTopLevelItem(item);
      e=e->getNextElementSibling();
    }
    return e;
  }

  DOMElement* PlotFeatureWidget::writeXMLFile2(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    for(size_t i=0; i<tree->topLevelItemCount(); i++) {
      DOMElement *ele = D(doc)->createElement(specialType);
      E(ele)->setAttribute("value",NamespaceURI(tree->topLevelItem(i)->text(3).toStdString())%tree->topLevelItem(i)->text(1).toStdString());
      ele->insertBefore(doc->createTextNode(X()%tree->topLevelItem(i)->text(2).toStdString()), nullptr);
      parent->insertBefore(ele, ref);
    }
    return nullptr;
  }

  CloneWidget::CloneWidget() {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    count = new ExtWidget("Count",new PhysicalVariableWidget(new ScalarWidget("1")));
    layout->addWidget(count);
    counterName = new ExtWidget("Countername",new TextWidget("n"));
    connect(counterName->getWidget(),&TextWidget::widgetChanged,this,&CloneWidget::widgetChanged);
    layout->addWidget(counterName);
    onlyif = new ExtWidget("Onlyif",new PhysicalVariableWidget(new ScalarWidget("1")));
    layout->addWidget(onlyif);
  }

  void CloneWidget::setCount(const QString &count_) {
    static_cast<PhysicalVariableWidget*>(count->getWidget())->setValue(count_);
  }

  QString CloneWidget::getCount() const {
    return static_cast<PhysicalVariableWidget*>(count->getWidget())->getValue();
  }

  void CloneWidget::setCounterName(const QString &counterName_) {
    static_cast<TextWidget*>(counterName->getWidget())->setText(counterName_);
  }

  QString CloneWidget::getCounterName() const {
    return static_cast<TextWidget*>(counterName->getWidget())->getText();
  }

  void CloneWidget::setOnlyif(const QString &onlyif_) {
    static_cast<PhysicalVariableWidget*>(onlyif->getWidget())->setValue(onlyif_);
  }

  QString CloneWidget::getOnlyif() const {
    return static_cast<PhysicalVariableWidget*>(onlyif->getWidget())->getValue();
  }

  XMLEditorWidget::XMLEditorWidget(const QString &text) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    edit = new QTextEdit;
    edit->setMinimumSize(300,200);
    setText(text);
    new XMLHighlighter(edit->document());
    static const QFont fixedFont=QFontDatabase::systemFont(QFontDatabase::FixedFont);
    edit->setFont(fixedFont);
    layout->addWidget(edit);
  }

  DOMElement* XMLEditorWidget::initializeUsingXML(DOMElement *element) {
    string text = X()%mw->serializer->writeToString(element);
    edit->setText(QString::fromStdString(text));
    return element;
  }

  DOMElement* XMLEditorWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMLSInput *source = mw->impl->createLSInput();
    X x;
    source->setStringData(x%edit->toPlainText().toStdString());
    try {
      DOMElement *element;
      if(dynamic_cast<DOMDocument*>(parent->getParentNode())) {
        DOMDocument *doc = mw->parser->parse(source);
        DOMElement *ele = doc->getDocumentElement();
        element = static_cast<xercesc::DOMElement*>(parent->getOwnerDocument()->importNode(ele,true));
        element->getOwnerDocument()->replaceChild(element, parent);
      }
      else
        element = static_cast<xercesc::DOMElement*>(mw->parser->parseWithContext(source, parent, DOMLSParser::ACTION_REPLACE));
      return element;
    }
    catch(DOMLSException &ex) {
      mw->setExitBad();
      cerr << X()%ex.msg << endl;
    }
    return nullptr;
  }

  ExtStringWidget::ExtStringWidget(Element *element_) : element(element_) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    value = new ChoiceWidget(new StringWidgetFactory(""),QBoxLayout::RightToLeft,5);
    layout->addWidget(value);
    if(element) {
      QPushButton *button = new QPushButton(tr("Browse"));
      layout->addWidget(button);
      eleBrowser = new ElementBrowser<Element>(nullptr,this);
      connect(eleBrowser,&BasicElementBrowser::accepted,this,&ExtStringWidget::setElement);
      connect(button,&QPushButton::clicked,this,&ExtStringWidget::showBrowser);
    }
  }

  void ExtStringWidget::showBrowser() {
    QString val = static_cast<PhysicalVariableWidget*>(value->getWidget())->getValue();
    eleBrowser->setSelection(static_cast<Element*>(element)->getByPath<Element>(val.mid(1,val.size()-2)));
    eleBrowser->show();
  }

  void ExtStringWidget::setElement() {
    Element *selectedElement = eleBrowser->getSelection();
    static_cast<PhysicalVariableWidget*>(value->getWidget())->setValue((selectedElement and selectedElement->getParent())?"\""+selectedElement->getXMLPath(static_cast<Element*>(element),true)+"\"":"");
  }

  StateWidget::StateWidget() {
    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);

    tree = new QTreeWidget;
    QStringList labels;
    labels << "Name" << "Value";
    tree->setHeaderLabels(labels);
    layout->addWidget(tree,0,0,4,1);
    tree->setColumnWidth(0,150);
    tree->setColumnWidth(1,50);

    dialog = new StateDialog(this);
    dialog->setModal(true);

    connect(dialog, &StateDialog::accepted, this, &StateWidget::updateTreeItem);

    QPushButton *add = new QPushButton("Add");
    connect(add,&QPushButton::clicked,this,&StateWidget::addState);
    layout->addWidget(add,0,1);

    QPushButton *update = new QPushButton("Edit");
    connect(update,&QPushButton::clicked,this,&StateWidget::editState);
    connect(tree, &QTreeWidget::itemDoubleClicked, this, &StateWidget::editState);
    layout->addWidget(update,1,1);

    QPushButton *remove = new QPushButton("Remove");
    connect(remove,&QPushButton::clicked,this,&StateWidget::removeState);
    layout->addWidget(remove,2,1);

    layout->setColumnStretch(0,10);
  }

  vector<QString> StateWidget::getNames() const {
    vector<QString> names;
    for(size_t i=0; i<tree->topLevelItemCount(); i++)
      names.push_back(tree->topLevelItem(i)->text(0));
    return names;
  }

  void StateWidget::updateTreeItem() {
    QTreeWidgetItem *item = tree->currentItem();
    if(item) {
      item->setText(0, dialog->getName());
      item->setText(1, dialog->getValue());
      emit widgetChanged();
    }
  }

  void StateWidget::addState() {
    auto *item = new QTreeWidgetItem;
    tree->addTopLevelItem(item);
    tree->setCurrentItem(item);
    editState();
  }

  void StateWidget::editState() {
    QTreeWidgetItem *item = tree->currentItem();
    if(item) {
      dialog->setName(item->text(0));
      dialog->setValue(item->text(1));
      dialog->show();
    }
  }

  void StateWidget::removeState() {
    tree->takeTopLevelItem(tree->indexOfTopLevelItem(tree->currentItem()));
  }

  DOMElement* StateWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"state");
    while(e && (E(e)->getTagName()==MBSIMCONTROL%"state")) {
      auto *item = new QTreeWidgetItem;
      item->setText(0, QString::fromStdString(E(e)->getAttributeQName("name").second));
      item->setText(1, QString::fromStdString(E(e)->getAttributeQName("value").second));
      tree->addTopLevelItem(item);
      e=e->getNextElementSibling();
    }
    return e;
  }

  DOMElement* StateWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    for(size_t i=0; i<tree->topLevelItemCount(); i++) {
      DOMElement *ele = D(doc)->createElement(MBSIMCONTROL%"state");
      E(ele)->setAttribute("name",tree->topLevelItem(i)->text(0).toStdString());
      E(ele)->setAttribute("value",tree->topLevelItem(i)->text(1).toStdString());
      parent->insertBefore(ele, ref);
    }
    return nullptr;
  }

  QString StateWidget::getXMLComment(DOMElement *element) {
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"state");
    if(e) {
      auto *cele = E(e)->getFirstCommentChild();
      if(cele)
	return (QString::fromStdString(X()%cele->getNodeValue()));
    }
    return "";
  }

  void StateWidget::setXMLComment(const QString &comment, DOMNode *element) {
    DOMElement *e=E(static_cast<DOMElement*>(element))->getFirstElementChildNamed(MBSIMCONTROL%"state");
    if(e) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      auto *cele = E(static_cast<DOMElement*>(e))->getFirstCommentChild();
      if(cele)
	cele->setData(X()%comment.toStdString());
      else
	e->insertBefore(doc->createComment(X()%comment.toStdString()), e->getFirstChild());
    }
  }

  TransitionWidget::TransitionWidget(Element *element) {
    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);

    tree = new QTreeWidget;
    QStringList labels;
    labels << "Source" << "Destination" << "Signal" << "Threshold";
    tree->setHeaderLabels(labels);
    layout->addWidget(tree,0,0,4,1);
    tree->setColumnWidth(0,150);
    tree->setColumnWidth(1,150);
    tree->setColumnWidth(2,250);
    tree->setColumnWidth(3,50);

    dialog = new TransitionDialog(element,this);
    dialog->setModal(true);

    connect(dialog, &TransitionDialog::accepted, this, &TransitionWidget::updateTreeItem);

    QPushButton *add = new QPushButton("Add");
    connect(add,&QPushButton::clicked,this,&TransitionWidget::addTransition);
    layout->addWidget(add,0,1);

    QPushButton *update = new QPushButton("Edit");
    connect(update,&QPushButton::clicked,this,&TransitionWidget::editTransition);
    connect(tree, &QTreeWidget::itemDoubleClicked, this, &TransitionWidget::editTransition);
    layout->addWidget(update,1,1);

    QPushButton *remove = new QPushButton("Remove");
    connect(remove,&QPushButton::clicked,this,&TransitionWidget::removeTransition);
    layout->addWidget(remove,2,1);

    layout->setColumnStretch(0,10);
  }

  void TransitionWidget::updateTreeItem() {
    QTreeWidgetItem *item = tree->currentItem();
    if(item) {
      item->setText(0, dialog->getSource());
      item->setText(1, dialog->getDestination());
      item->setText(2, dialog->getSignal());
      item->setText(3, dialog->getThreshold());
    }
  }

  void TransitionWidget::addTransition() {
    auto *item = new QTreeWidgetItem;
    tree->addTopLevelItem(item);
    tree->setCurrentItem(item);
    editTransition();
  }

  void TransitionWidget::editTransition() {
    QTreeWidgetItem *item = tree->currentItem();
    if(item) {
      dialog->setSource(item->text(0));
      dialog->setDestination(item->text(1));
      dialog->setSignal(item->text(2));
      dialog->setThreshold(item->text(3));
      dialog->show();
    }
  }

  void TransitionWidget::removeTransition() {
    tree->takeTopLevelItem(tree->indexOfTopLevelItem(tree->currentItem()));
  }

  void TransitionWidget::setStringList(const vector<QString> &list) {
    dialog->setStringList(list);
  }

  DOMElement* TransitionWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"transition");
    while(e && (E(e)->getTagName()==MBSIMCONTROL%"transition")) {
      auto *item = new QTreeWidgetItem;
      item->setText(0, QString::fromStdString(E(e)->getAttributeQName("source").second));
      item->setText(1, QString::fromStdString(E(e)->getAttributeQName("destination").second));
      item->setText(2, QString::fromStdString(E(e)->getAttributeQName("signal").second));
      item->setText(3, QString::fromStdString(E(e)->getAttributeQName("threshold").second));
      tree->addTopLevelItem(item);
      e=e->getNextElementSibling();
    }
    return e;
  }

  DOMElement* TransitionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    for(size_t i=0; i<tree->topLevelItemCount(); i++) {
      DOMElement *ele = D(doc)->createElement(MBSIMCONTROL%"transition");
      E(ele)->setAttribute("source",tree->topLevelItem(i)->text(0).toStdString());
      E(ele)->setAttribute("destination",tree->topLevelItem(i)->text(1).toStdString());
      E(ele)->setAttribute("signal",tree->topLevelItem(i)->text(2).toStdString());
      E(ele)->setAttribute("threshold",tree->topLevelItem(i)->text(3).toStdString());
      parent->insertBefore(ele, ref);
    }
    return nullptr;
  }

  QString TransitionWidget::getXMLComment(DOMElement *element) {
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"transition");
    if(e) {
      auto *cele = E(e)->getFirstCommentChild();
      if(cele)
	return (QString::fromStdString(X()%cele->getNodeValue()));
    }
    return "";
  }

  void TransitionWidget::setXMLComment(const QString &comment, DOMNode *element) {
    DOMElement *e=E(static_cast<DOMElement*>(element))->getFirstElementChildNamed(MBSIMCONTROL%"transition");
    if(e) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      auto *cele = E(static_cast<DOMElement*>(e))->getFirstCommentChild();
      if(cele)
	cele->setData(X()%comment.toStdString());
      else
	e->insertBefore(doc->createComment(X()%comment.toStdString()), e->getFirstChild());
    }
  }

  CommentWidget::CommentWidget() {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    edit = new QPlainTextEdit;
    layout->addWidget(edit);
  }

  void CommentWidget::setComment(const QString &comment) {
    edit->setPlainText(comment);
  }

  QString CommentWidget::getComment() const {
    return edit->toPlainText();
  }

  DOMElement* CommentWidget::initializeUsingXML(DOMElement *element) {
    auto *cele = E(element)->getFirstCommentChild();
    if(cele) {
      setComment(QString::fromStdString(X()%cele->getNodeValue()));
      return element;
    }
    return nullptr;
  }

  DOMElement* CommentWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    QString text = getComment();
    if(not text.isEmpty()) {
      auto *cele = E(static_cast<DOMElement*>(parent))->getFirstCommentChild();
      if(cele)
	cele->setData(X()%text.toStdString());
      else
	parent->insertBefore(parent->getOwnerDocument()->createComment(X()%text.toStdString()), parent->getFirstChild());
    }
    return nullptr;
  }

}
