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

#include <config.h>
#include "dialogs.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "rigid_body.h"
#include "signal_.h"
#include "constraint.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "mainwindow.h"
#include "octave_utils.h"
#include "data_plot.h"
#include "element_view.h"
#include "treemodel.h"
#include "treeitem.h"
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QTreeWidget>
#include <QTableWidget>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QMessageBox>
#include <QFileInfo>
#include <QSettings>
#include <qwt_plot.h>
#include <boost/math/constants/constants.hpp>

using namespace std;
using namespace boost::math::constants;

namespace MBSimGUI {

  extern MainWindow *mw;

  QModelIndex findTreeItemData(QModelIndex root, TreeItemData *sel) {
    auto *model = static_cast<const ElementTreeModel*>(root.model());
    for(int i=0; i<model->rowCount(root); i++) {
      if(model->getItem(root.child(i,0))->getItemData()==sel)
        return root.child(i,0);
      QModelIndex index = findTreeItemData(root.child(i,0), sel);
      if(index.isValid())
        return index;
    }
    return QModelIndex();
  }

  EvalDialog::EvalDialog(const vector<vector<QString> > &var_, QWidget *parent) : QDialog(parent) {
    var.resize(var_.size());
    for(int i=0; i<var_.size(); i++) {
      var[i].resize(var_[i].size());
      for(int j=0; j<var[i].size(); j++)
        var[i][j] = var_[i][j].toDouble();
    }

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto *layout = new QGridLayout;
    mainlayout->addLayout(layout);

    layout->addWidget(new QLabel("Format:"),0,0);
    format = new QComboBox;
    format->addItems(QStringList() << "e" << "E" << "f" << "g" << "G");
    format->setCurrentIndex(3);
    layout->addWidget(format,0,1);
    connect(format, SIGNAL(currentIndexChanged(int)), this, SLOT(updateWidget()));

    layout->addWidget(new QLabel("Precision:"),0,2);
    precision = new QSpinBox;
    precision->setValue(6);
    layout->addWidget(precision,0,3);
    connect(precision, SIGNAL(valueChanged(int)), this, SLOT(updateWidget()));

    tab = new QTableWidget;
    tab->setRowCount(var.size());
    tab->setColumnCount(!var.empty()?var[0].size():0);
    for(int i=0; i<tab->rowCount(); i++) {
      for(int j=0; j<tab->columnCount(); j++)
        tab->setItem(i,j,new QTableWidgetItem(QString::number(var[i][j],'g',6)));
    }

    layout->addWidget(tab,1,0,1,5);

    auto *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Close);
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

    mainlayout->addWidget(buttonBox);

    layout->setColumnStretch(4, 10);
//    layout->setColumnStretch(2, 20);

    setWindowTitle("Expression evaluation");
  }

  void EvalDialog::updateWidget() {
    QString f = format->currentText();
    int p = precision->value();
    for(int i=0; i<tab->rowCount(); i++) {
      for(int j=0; j<tab->columnCount(); j++)
        tab->item(i,j)->setText(QString::number(var[i][j],f[0].toLatin1(),p));
    }
  }

  BasicElementBrowser::BasicElementBrowser(Element* selection_, const QString &name, QWidget *parent) : QDialog(parent), selection(selection_) {
    auto* mainLayout=new QGridLayout;
    setLayout(mainLayout);
    eleList = new QTreeView;
    eleList->setModel(mw->getElementView()->model());
    eleList->setColumnWidth(0,250);
    eleList->setColumnWidth(1,200);
    eleList->hideColumn(1);
    mainLayout->addWidget(eleList,0,0);
    connect(eleList->selectionModel(),SIGNAL(currentChanged(const QModelIndex&,const QModelIndex&)), this, SLOT(selectionChanged(const QModelIndex&)));

    okButton = new QPushButton("Ok");
    if(!selection)
      okButton->setDisabled(true);
    mainLayout->addWidget(okButton,1,0);
    connect(okButton, SIGNAL(clicked(bool)), this, SLOT(accept()));

    QPushButton *button = new QPushButton("Cancel");
    mainLayout->addWidget(button,1,1);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(reject()));

    setWindowTitle(name+" browser");
  }

  void BasicElementBrowser::showEvent(QShowEvent *event) {
    QSettings settings;
    restoreGeometry(settings.value("basicelementbrowser/geometry").toByteArray());
    QDialog::showEvent(event);
    oldID = mw->getHighlightedObject();
    QModelIndex index1 = findTreeItemData(eleList->model()->index(0,0),selection);
    QModelIndex index2 = mw->getElementView()->selectionModel()->currentIndex().parent().parent();
    eleList->setCurrentIndex(index1.isValid()?index1:index2);
    if(selection) mw->highlightObject(selection->getID());
  }

  void BasicElementBrowser::hideEvent(QHideEvent *event) {
    QSettings settings;
    settings.setValue("basicelementbrowser/geometry", saveGeometry());
    QDialog::hideEvent(event);
    mw->highlightObject(oldID);
  }

  void BasicElementBrowser::selectionChanged(const QModelIndex &current) {
    auto *model = static_cast<ElementTreeModel*>(eleList->model());
    auto *element = dynamic_cast<Element*>(model->getItem(current)->getItemData());
    if(checkForElement(element)) {
      selection = element;
      mw->highlightObject(element->getID());
      okButton->setDisabled(false);
    }
    else {
      mw->highlightObject(0);
      okButton->setDisabled(true);
    }
  }

  SaveDialog::SaveDialog(QWidget *parent) : QDialog(parent) {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    parameter = new QCheckBox("Include parameters");
    layout->addWidget(parameter);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Ok);
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    layout->addWidget(buttonBox);
  }

  EigenanalysisDialog::EigenanalysisDialog(const QString &name, QWidget *parent) : QDialog(parent) {
    OctaveParser parser(name.toStdString());
    parser.parse();
    fmatvec::Vector<fmatvec::Var,complex<double> > w = static_cast<const OctaveComplexMatrix*>(parser.get(0))->get<fmatvec::Vector<fmatvec::Var,complex<double> > >();
    fmatvec::SquareMatrix<fmatvec::Var,complex<double> > V = static_cast<const OctaveComplexMatrix*>(parser.get(1))->get<fmatvec::SquareMatrix<fmatvec::Var,complex<double> > >();

    std::vector<std::pair<double,int> > f;
    for (int i=0; i<w.size(); i++) {
      if((abs(imag(w(i))) > 1e-13) and (i < w.size()-1) and (w(i+1)==conj(w(i)))) {
        f.push_back(pair<double,int>(imag(w(i))/2/pi<double>(),i));
        i++;
      }
    }
    std::sort(f.begin(), f.end());

    QGridLayout *layout = new QGridLayout;
    setLayout(layout);
    table = new QTableWidget(f.size(),5);
    QStringList labels;
    labels << "Mode" << "Frequency" << "Exponential decay" << "Angular frequency" << "Damping ratio";
    table->setHorizontalHeaderLabels(labels);
    layout->addWidget(table,0,0);
    int n = V.rows()/2;
    QVector<double> m(n);
    QVector<QVector<double> > A(f.size(),QVector<double>(n));
    for(int k=0; k<n; k++)
      m[k] = k+1;
    for(int i=0; i<f.size(); i++) {
      int j = f[i].second;
      table->setItem(i, 0, new QTableWidgetItem(QString::number(i+1)));
      table->setItem(i, 1, new QTableWidgetItem(QString::number(f[i].first)));
      table->setItem(i, 2, new QTableWidgetItem(QString::number(-w(j).real())));
      table->setItem(i, 3, new QTableWidgetItem(QString::number(w(j).imag())));
      table->setItem(i, 4, new QTableWidgetItem(QString::number(-w(j).real()/w(j).imag())));
      double max = fabs(V(n,j).real());
      int ind = 0;
      for(int k=1; k<n; k++) {
        if(fabs(V(n+k,j).real())>max) {
          max = fabs(V(n+k,j).real());
          ind= k;
        }
      }
      for(int k=0; k<n; k++)
        A[i][k] = V(k+n,j).real()/V(ind+n,j).real();
    }
    if(f.size()) {
      table->selectRow(0);
      plot = new DataPlot(m,A,"Mode", "Eigenmode", "DOF", "-", this);
      plot->setSymbol(QwtSymbol::Diamond,10);
      plot->setAxisScale(QwtPlot::xBottom,1-0.1,n+0.1,1);
      plot->setAxisScale(QwtPlot::yLeft,-1.1,1.1);
      plot->replot();
      layout->addWidget(plot,0,1);
      QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
      buttonBox->addButton(QDialogButtonBox::Ok);
      layout->addWidget(buttonBox,1,0,1,2);
      connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
      connect(plot, SIGNAL(numChanged(int)), this, SLOT(selectRow(int)));
      connect(table, SIGNAL(cellClicked(int,int)), this, SLOT(selectMode(int,int)));
    }
  }

  void EigenanalysisDialog::selectRow(int i) {
    table->blockSignals(true);
    table->selectRow(i-1);
    table->blockSignals(false);
  }

  void EigenanalysisDialog::selectMode(int row, int col) {
    plot->blockSignals(true);
    plot->changeNum(row+1);
    plot->blockSignals(false);
  }

  HarmonicResponseDialog::HarmonicResponseDialog(const QString &name, QWidget *parent) : QDialog(parent) {
    OctaveParser parser(name.toStdString());
    parser.parse();
    fmatvec::MatV t_ = static_cast<const OctaveMatrix*>(parser.get(1))->get<fmatvec::MatV>();
    fmatvec::MatV A_ = static_cast<const OctaveMatrix*>(parser.get(2))->get<fmatvec::MatV>();
    QVector<double> t(t_.rows());
    QVector<QVector<double> > A(A_.cols(),QVector<double>(A_.rows()));
    for(int i=0; i<t_.rows(); i++) {
      t[i] = t_(i,0);
      for(int j=0; j<A_.cols(); j++)
        A[j][i] = A_(i,j);
    }

    QGridLayout *layout = new QGridLayout;
    setLayout(layout);
    DataPlot *plot = new DataPlot(t,A,"DOF", "Frequency response", "f in Hz", "A", this);
    plot->replot();
    layout->addWidget(plot,0,0);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Ok);
    layout->addWidget(buttonBox,1,0,1,1);
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  }

  SourceDialog::SourceDialog(Element *element, QWidget *parent) : QDialog(parent) {
    setWindowTitle(QString("XML View"));
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    XMLEditorWidget *edit = new XMLEditorWidget;
    edit->initializeUsingXML(element->getXMLElement());
    layout->addWidget(edit);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Ok);
    layout->addWidget(buttonBox);
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  }

}
