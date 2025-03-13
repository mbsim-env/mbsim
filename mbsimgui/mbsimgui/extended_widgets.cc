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
#include "extended_widgets.h"
#include "variable_widgets.h"
#include "custom_widgets.h"
#include "unknown_widget.h"
#include "mainwindow.h"
#include <QListWidget>
#include <QStackedWidget>
#include <QDialogButtonBox>
#include <xercesc/dom/DOMComment.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  bool MouseEvent::eventFilter(QObject *obj, QEvent *event) {
    if(event->type() == QEvent::MouseButtonPress) {
      emit mouseButtonPressed();
      return true;
    }
    else
      return QObject::eventFilter(obj, event);
  }

  optional<QPixmap> ExtWidget::expandedPixmap, ExtWidget::collapsedPixmap;
  optional<QIcon> ExtWidget::commentIcon, ExtWidget::noCommentIcon;

  ExtWidget::ExtWidget(const QString &name, Widget *widget_, bool checkable_, bool active, FQN xmlName_, bool comment,
                       const QString &defaultEmployed) : 
                       checkable(checkable_), checked(active), widget(widget_), xmlName(std::move(xmlName_)) {
    if(xmlName!=FQN()) comment = true;
    if(!expandedPixmap) {
      auto iconPath(MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons");
      QFontInfo fontinfo(font());
      expandedPixmap = Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())).pixmap(fontinfo.pixelSize(),fontinfo.pixelSize());
      collapsedPixmap = Utils::QIconCached(QString::fromStdString((iconPath/"collapsed.svg").string())).pixmap(fontinfo.pixelSize(),fontinfo.pixelSize());
      commentIcon = Utils::QIconCached(QString::fromStdString((iconPath/"comment.svg").string()));
      noCommentIcon = Utils::QIconCached(QString::fromStdString((iconPath/"nocomment.svg").string()));
    }

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    checkable = checkable_;
    checked = active;
    if(checkable) {
      auto *expandableWidget = new QWidget;
      expandableWidget->setCursor(Qt::PointingHandCursor);
      expandableWidget->setContentsMargins(0,0,0,0);
      auto *expandableLayout = new QHBoxLayout;
      expandableLayout->setContentsMargins(0,0,0,0);
      expandableWidget->setLayout(expandableLayout);
      iconLabel = new QLabel;
      iconLabel->setPixmap(checked ? *expandedPixmap : *collapsedPixmap);
      expandableLayout->addWidget(iconLabel);
      expandableLayout->addWidget(new QLabel(name));
      defaultLabel = new QLabel(defaultEmployed);
      defaultLabel->setDisabled(true);
      expandableLayout->addWidget(defaultLabel);
      defaultLabel->setVisible(not active);
      expandableLayout->setStretch(0,0);
      expandableLayout->setStretch(1,0);
      expandableLayout->setStretch(2,0);
      if(comment) {
	commentButton = new QPushButton;
	expandableLayout->addWidget(commentButton);
	commentButton->setVisible(active);
      }
      expandableLayout->addStretch();
      layout->addWidget(expandableWidget);
      layout->addWidget(widget);
      widget->setContentsMargins(10,0,0,0);
      MouseEvent *mouseEvent = new MouseEvent(expandableWidget);
      expandableWidget->installEventFilter(mouseEvent);
      expandableWidget->setToolTip("Click to define or remove this property");
      widget->setVisible(active);
      connect(mouseEvent,&MouseEvent::mouseButtonPressed,this,[=]{
          setActive(not checked);
          emit widgetChanged();
          emit clicked(checked);
          });
    }
    else {
      auto *hboxw = new QWidget;
      hboxw->setContentsMargins(0,0,0,0);
      auto *hboxl = new QHBoxLayout;
      hboxl->setContentsMargins(0,0,0,0);
      hboxw->setLayout(hboxl);
      auto *label = new QLabel;
      hboxl->addWidget(label);
      if(comment) {
        commentButton = new QPushButton;
        hboxl->addWidget(commentButton);
      }
      hboxl->addStretch();
      layout->addWidget(hboxw);
      layout->addWidget(widget);
      widget->setContentsMargins(10,0,0,0);
      label->setToolTip("This property must be defined");
      label->setText(name);
    }
    if(commentButton) {
      QFontInfo fontinfo(font());
      QSize size(fontinfo.pixelSize(),fontinfo.pixelSize());
      commentButton->setIconSize(size);
      commentButton->setFixedSize(size);
      commentButton->setFlat(true);
      commentButton->setCursor(Qt::WhatsThisCursor);
      setComment("");
      connect(commentButton, &QPushButton::clicked, this, &ExtWidget::editComment);
    }
    connect(widget,&Widget::widgetChanged,this,&ExtWidget::widgetChanged);
  }

  void ExtWidget::setActive(bool active) {
    if(checkable) {
      checked = active;
      iconLabel->setPixmap(checked ? *expandedPixmap : *collapsedPixmap);
      defaultLabel->setVisible(not checked);
      widget->setVisible(checked);
      if(commentButton) commentButton->setVisible(checked);
    }
  }

  DOMElement* ExtWidget::initializeUsingXML(DOMElement *element) {
    bool active = false;
    if(xmlName!=FQN()) {
      DOMElement *e=E(element)->getFirstElementChildNamed(xmlName);
      if(e) {
        active = widget->initializeUsingXML(e);
	auto *cele = E(e)->getFirstCommentChild();
	if(cele)
	  setComment(QString::fromStdString(X()%cele->getNodeValue()));
      }
    }
    else {
      active = widget->initializeUsingXML(element);
      if(commentButton)
	setComment(widget->getXMLComment(element));
    }
    setActive(active);
    return active?element:nullptr;
  }

  DOMElement* ExtWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele = nullptr;
    if(xmlName!=FQN()) {
      if(isActive()) {
	DOMDocument *doc = parent->getOwnerDocument();
	DOMElement *newele = D(doc)->createElement(xmlName);
        parent->insertBefore(newele,ref);
        ele = widget->writeXMLFile(newele);
	if(not comment.isEmpty()) {
	  auto *cele = E(static_cast<DOMElement*>(newele))->getFirstCommentChild();
	  if(cele)
	    cele->setData(X()%comment.toStdString());
	  else
	    newele->insertBefore(doc->createComment(X()%comment.toStdString()), newele->getFirstChild());
	}
      }
    }
    else {
      if(isActive()) {
	ele = widget->writeXMLFile(parent,ref);
	if(commentButton and (not comment.isEmpty()))
	  widget->setXMLComment(comment,parent);
      }
    }
    return ele;
  }

  void ExtWidget::setComment(const QString &commentStr) {
    if(commentStr.isEmpty()) {
      commentButton->setIcon(*noCommentIcon);
      commentButton->setToolTip("Click to add a comment");
      comment = "";
    }
    else {
      commentButton->setIcon(*commentIcon);
      commentButton->setToolTip(commentStr);
      comment = commentStr;
    }
  }

  void ExtWidget::editComment() {
    QDialog dialog(this);
    auto *layout = new QVBoxLayout;
    dialog.setLayout(layout);
    auto *commentWidget = new CommentWidget;
    commentWidget->setComment(comment);
    layout->addWidget(commentWidget);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Ok);
    buttonBox->addButton(QDialogButtonBox::Cancel);
    layout->addWidget(buttonBox);
    connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    auto res = dialog.exec();
    if(res==QDialog::Accepted)
      setComment(commentWidget->getComment());
  }

  ChoiceWidget::ChoiceWidget(WidgetFactory *factory_, QBoxLayout::Direction dir, int mode_) : widget(nullptr), factory(factory_), mode(mode_) {
    layout = new QBoxLayout(dir);
    layout->setMargin(0);
    setLayout(layout);

    comboBox = new CustomComboBox;
    for(int i=0; i<factory->getSize(); i++)
      comboBox->addItem(factory->getName(i));
    layout->addWidget(comboBox);
    comboBox->setCurrentIndex(factory->getDefaultIndex());
    defineWidget(factory->getDefaultIndex());
    connect(comboBox,QOverload<int>::of(&CustomComboBox::currentIndexChanged),this,&ChoiceWidget::defineWidget);
    connect(comboBox,QOverload<int>::of(&CustomComboBox::currentIndexChanged),this,&ChoiceWidget::comboChanged);
  }

  void ChoiceWidget::setWidgetFactory(WidgetFactory *factory_) {
    factory = factory_;
    comboBox->blockSignals(true);
    comboBox->clear();
    for(int i=0; i<factory->getSize(); i++)
      comboBox->addItem(factory->getName(i));
    comboBox->setCurrentIndex(factory->getDefaultIndex());
    defineWidget(factory->getDefaultIndex());
    comboBox->blockSignals(false);
  }

  void ChoiceWidget::defineWidget(int index) {
    if(widget) {
      layout->removeWidget(widget);
      delete widget;
    }
    widget = factory->createWidget(index);
    if(layout->direction()==QBoxLayout::TopToBottom)
      widget->setContentsMargins(factory->getMargin(),0,0,0);
    layout->addWidget(widget, getStretchHint());
    connect(widget,&Widget::widgetChanged,this,&ChoiceWidget::widgetChanged);
    emit widgetChanged();
  }

  DOMElement* ChoiceWidget::initializeUsingXML(DOMElement *element) {
    if (mode<=1) {
      DOMElement *e=(mode==0)?element->getFirstElementChild():element;
      if(e) {
        int k = factory->getFallbackIndex();
        for(int i=0; i<factory->getSize(); i++) {
          if(E(e)->getTagName()==factory->getXMLName(i)) {
            k = i;
            break;
          }
        }
        blockSignals(true);
        defineWidget(k);
        blockSignals(false);
        comboBox->blockSignals(true);
        comboBox->setCurrentIndex(k);
        comboBox->blockSignals(false);
        return widget->initializeUsingXML(e);
      }
    }
    else if (mode<=3) {
      for(int i=0; i<factory->getSize(); i++) {
        DOMElement *e=E(element)->getFirstElementChildNamed(factory->getXMLName(i));
        if(e) {
          blockSignals(true);
          defineWidget(i);
          blockSignals(false);
          comboBox->blockSignals(true);
          comboBox->setCurrentIndex(i);
          comboBox->blockSignals(false);
          return widget->initializeUsingXML(e);
        }
      }
    }
    else {
      for(int i=0; i<factory->getSize(); i++) {
        blockSignals(true);
        defineWidget(i);
        blockSignals(false);
        comboBox->blockSignals(true);
        comboBox->setCurrentIndex(i);
        comboBox->blockSignals(false);
        if(widget->initializeUsingXML(element))
          return element;
      }
    }
    return nullptr;
  }

  DOMElement* ChoiceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    if(mode==3) {
      xercesc::DOMDocument *doc=parent->getOwnerDocument();
      DOMElement *ele0 = D(doc)->createElement(factory->getXMLName(getIndex()));
      widget->writeXMLFile(ele0);
      parent->insertBefore(ele0,ref);
    }
    else
      widget->writeXMLFile(parent,ref);
    return nullptr;
  }

  QString ChoiceWidget::getXMLComment(DOMElement *element) {
    if(mode<=3) {
      DOMElement *e=E(element)->getFirstElementChildNamed(factory->getXMLName(getIndex()));
      if(e) {
	auto *cele = E(e)->getFirstCommentChild();
	if(cele)
	  return (QString::fromStdString(X()%cele->getNodeValue()));
      }
    }
    return "";
  }

  void ChoiceWidget::setXMLComment(const QString &comment, DOMNode *element) {
    if(mode==3) {
      DOMElement *e=E(static_cast<DOMElement*>(element))->getFirstElementChildNamed(factory->getXMLName(getIndex()));
      if(e) {
	xercesc::DOMDocument *doc=element->getOwnerDocument();
	auto *cele = E(static_cast<DOMElement*>(e))->getFirstCommentChild();
	if(cele)
	  cele->setData(X()%comment.toStdString());
	else
	  e->insertBefore(doc->createComment(X()%comment.toStdString()), e->getFirstChild());
      }
    }
  }


  ContainerWidget::ContainerWidget() {
    layout = new QVBoxLayout;
    setLayout(layout);
    layout->setMargin(0);
  }

  void ContainerWidget::resize_(int m, int n) {
    for(auto & i : widget)
      i->resize_(m,n);
  }

  void ContainerWidget::addWidget(Widget *widget_) {
    layout->addWidget(widget_); 
    widget.push_back(widget_);
    connect(widget[widget.size()-1],&Widget::widgetChanged,this,&ChoiceWidget::widgetChanged);
  }

  void ContainerWidget::updateWidget() {
    for(unsigned int i=0; i<widget.size(); i++)
      getWidget<Widget>(i)->updateWidget();
  }

  DOMElement* ContainerWidget::initializeUsingXML(DOMElement *element) {
    bool flag = false;
    for(unsigned int i=0; i<widget.size(); i++)
      if(getWidget<Widget>(i)->initializeUsingXML(element))
        flag = true;
    return flag?element:nullptr;
  }

  DOMElement* ContainerWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    for(unsigned int i=0; i<widget.size(); i++)
      getWidget<Widget>(i)->writeXMLFile(parent,ref);
    return nullptr;
  }

  ListWidget::ListWidget(WidgetFactory *factory_, const QString &name_, int m, int mode_, bool fixedSize, int minSize, int maxSize) : factory(factory_), name(name_), mode(mode_) {
    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);

    list = new QListWidget;
    list->setMinimumSize(200,200);
    layout->addWidget(list,0,0,2,1);
    stackedWidget = new QStackedWidget;
    layout->addWidget(stackedWidget,2,0,1,3);

    spinBox = new CustomSpinBox;
    spinBox->setDisabled(fixedSize);
    spinBox->setRange(minSize,maxSize);
    spinBox->setValue(m);
    layout->addWidget(spinBox,0,1);

    layout->setColumnStretch(0,10);

    changeSize(m);

    connect(list,&QListWidget::currentRowChanged,this,&ListWidget::changeCurrent);
    connect(spinBox,QOverload<int>::of(&CustomSpinBox::valueChanged),this,&ListWidget::changeSize);
  }

  ListWidget::~ListWidget() {
    delete factory;
  }

  int ListWidget::getSize() const {
    return stackedWidget->count();
  }

  void ListWidget::setSize(int m) {
    spinBox->setValue(m);
  }

  void ListWidget::setRange(int minSize, int maxSize) {
    spinBox->setRange(minSize,maxSize);
  }

  Widget* ListWidget::getWidgetVirtual(int i) const {
    return static_cast<Widget*>(stackedWidget->widget(i));
  }

  void ListWidget::changeSize(int size) {
    int n = size - list->count();
    if(n>0)
      addElements(n);
    else if(n<0)
      removeElements(-n);
  }

  void ListWidget::changeCurrent(int idx) {
    if(idx>=0) {
      if (stackedWidget->currentWidget() !=nullptr)
        stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      stackedWidget->setCurrentIndex(idx);
      stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      adjustSize();
    }
  }

  void ListWidget::resize_(int m, int n) {
    for(int i=0; i<stackedWidget->count(); i++)
      getWidget<Widget>(i)->resize_(m,n);
  }

  void ListWidget::addElements(int n, bool emitSignals) {

    int i = stackedWidget->count();

    for(int j=1; j<=n; j++) {
      list->addItem(name+" "+QString::number(i+j));

      Widget *widget = factory->createWidget();
      stackedWidget->addWidget(widget);
      if(i>0)
        widget->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

      connect(widget,&Widget::widgetChanged,this,&ListWidget::widgetChanged);
    }

    if(i==0)
      list->setCurrentRow(0);

    if(emitSignals)
      emit Widget::widgetChanged();
  }

  void ListWidget::removeElements(int n) {
    int N = stackedWidget->count();
    for(int j=0; j<n; j++) {
      auto *widget = stackedWidget->widget(N-j-1);
      stackedWidget->removeWidget(widget);
      delete widget;
      delete list->takeItem(N-j-1);
    }
    emit Widget::widgetChanged();
  }

  DOMElement* ListWidget::initializeUsingXML(DOMElement *element) {
    blockSignals(true);
    removeElements(getSize());
    blockSignals(false);
    list->blockSignals(true);
    spinBox->blockSignals(true);
    if(mode<=1) {
      DOMElement *e=(mode==0)?element->getFirstElementChild():element;
      while(e) {
        addElements(1,false);
        getWidget<Widget>(getSize()-1)->initializeUsingXML(e);
        e=e->getNextElementSibling();
      }
      spinBox->setValue(getSize());
    }
    else if(mode==2) {
      DOMElement *e=E(element)->getFirstElementChildNamed(factory->getXMLName());
      while(e and E(e)->getTagName()==factory->getXMLName()) {
        addElements(1,false);
        getWidget<Widget>(getSize()-1)->initializeUsingXML(e);
        e=e->getNextElementSibling();
      }
      spinBox->setValue(getSize());
    }
    else {
      DOMElement *e=E(element)->getFirstElementChildNamed(factory->getXMLName());
      while(e and E(e)->getTagName()==factory->getXMLName()) {
        addElements(1,false);
        e = getWidget<Widget>(getSize()-1)->initializeUsingXML(e);
      }
      spinBox->setValue(getSize());
    }
    list->blockSignals(false);
    spinBox->blockSignals(false);
    return element;
  }

  DOMElement* ListWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    if(mode<=1 or mode==3) {
      for(unsigned int i=0; i<getSize(); i++)
        getWidget<Widget>(i)->writeXMLFile(parent,ref);
    }
    else {
      xercesc::DOMDocument *doc=parent->getOwnerDocument();
      for(unsigned int i=0; i<getSize(); i++) {
        DOMElement *ele0 = D(doc)->createElement(factory->getXMLName());
        getWidget<Widget>(i)->writeXMLFile(ele0);
        parent->insertBefore(ele0,ref);
      }
    }
    return nullptr;
  }

  QString ListWidget::getXMLComment(DOMElement *element) {
    DOMElement *e;
    if(mode<=1 or mode==3)
      e = static_cast<DOMElement*>(element)->getFirstElementChild();
    else
      e = E(static_cast<DOMElement*>(element))->getFirstElementChildNamed(factory->getXMLName());
    if(e) {
      auto *cele = E(e)->getFirstCommentChild();
      if(cele)
	return (QString::fromStdString(X()%cele->getNodeValue()));
    }
    return "";
  }

  void ListWidget::setXMLComment(const QString &comment, DOMNode *element) {
    DOMElement *e;
    if(mode<=1 or mode==3)
      e = static_cast<DOMElement*>(element)->getFirstElementChild();
    else
      e = E(static_cast<DOMElement*>(element))->getFirstElementChildNamed(factory->getXMLName());
    if(e) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      auto *cele = E(static_cast<DOMElement*>(e))->getFirstCommentChild();
      if(cele)
	cele->setData(X()%comment.toStdString());
      else
	e->insertBefore(doc->createComment(X()%comment.toStdString()), e->getFirstChild());
    }
  }

  Widget* ChoiceWidgetFactory::createWidget(int i) {
    return new ChoiceWidget(factory,dir,mode);
  }

}
