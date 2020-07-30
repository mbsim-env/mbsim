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

  EvalDialog::EvalDialog(const vector<vector<QString>> &var_, int type_, QWidget *parent) : QDialog(parent), var(var_), varf(var_), type(type_) {

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto *layout = new QGridLayout;
    mainlayout->addLayout(layout);

    if(type==0) {
      layout->addWidget(new QLabel("Format:"),0,0);
      format = new QComboBox;
      format->addItems(QStringList() << "e" << "E" << "f" << "g" << "G");
      format->setCurrentIndex(3);
      layout->addWidget(format,0,1);
      connect(format, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &EvalDialog::updateWidget);

      layout->addWidget(new QLabel("Precision:"),0,2);
      precision = new QSpinBox;
      precision->setValue(6);
      layout->addWidget(precision,0,3);
      connect(precision, QOverload<int>::of(&QSpinBox::valueChanged), this, &EvalDialog::updateWidget);
    }

    formatVariables();

    tab = new QTableWidget;
    tab->setRowCount(varf.size());
    tab->setColumnCount(!varf.empty()?varf[0].size():0);
    for(int i=0; i<tab->rowCount(); i++) {
      for(int j=0; j<tab->columnCount(); j++)
        tab->setItem(i,j,new QTableWidgetItem(varf[i][j]));
    }

    layout->addWidget(tab,1,0,1,5);

    auto *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Close);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &EvalDialog::reject);

    mainlayout->addWidget(buttonBox);

    layout->setColumnStretch(4, 10);

    setWindowTitle("Expression evaluation");
  }

  void EvalDialog::formatVariables() {
    if(type==0) {
      QString f = format->currentText();
      int p = precision->value();
      for(int i=0; i<var.size(); i++) {
        for(int j=0; j<var[i].size(); j++)
          varf[i][j] = QString::number(var[i][j].toDouble(),f[0].toLatin1(),p);
      }
    }
  }

  void EvalDialog::updateWidget() {
    formatVariables();
    for(int i=0; i<tab->rowCount(); i++) {
      for(int j=0; j<tab->columnCount(); j++)
        tab->item(i,j)->setText(varf[i][j]);
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
    connect(eleList, &QTreeView::pressed, this, &BasicElementBrowser::selectionChanged);

    okButton = new QPushButton("Ok");
    if(!selection)
      okButton->setDisabled(true);
    mainLayout->addWidget(okButton,1,0);
    connect(okButton, &QPushButton::clicked, this, &BasicElementBrowser::accept);

    QPushButton *button = new QPushButton("Cancel");
    mainLayout->addWidget(button,1,1);
    connect(button, &QPushButton::clicked, this, &BasicElementBrowser::reject);

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
      mw->highlightObject("");
      okButton->setDisabled(true);
    }
  }

  EigenanalysisDialog::EigenanalysisDialog(QWidget *parent) : QDialog(parent) {
    setWindowTitle("Eigenanalysis");
    OctaveParser parser(mw->getUniqueTempDir().generic_string()+"/eigenanalysis.mat");
    parser.parse();
    fmatvec::Vector<fmatvec::Var,complex<double>> w = static_cast<const OctaveComplexMatrix*>(parser.get(0))->get<fmatvec::Vector<fmatvec::Var,complex<double>>>();
    fmatvec::SquareMatrix<fmatvec::Var,complex<double>> V = static_cast<const OctaveComplexMatrix*>(parser.get(1))->get<fmatvec::SquareMatrix<fmatvec::Var,complex<double>>>();

    std::vector<std::pair<double,int>> f;
    for (int i=0; i<w.size(); i++) {
      if((abs(imag(w(i))) > 1e-13) and (i < w.size()-1) and (w(i+1)==conj(w(i)))) {
        f.push_back(pair<double,int>(imag(w(i))/2/pi<double>(),i));
        i++;
      }
    }
    std::sort(f.begin(), f.end());

    ifstream is(mw->getUniqueTempDir().generic_string()+"/statetable.asc");
    QVector<QString> name;
    QVector<int> number;
    string name_;
    char label_;
    int number_;
    while(is) {
      is >> name_ >> label_ >> number_;
      if(label_!='q')
        break;
      name.append(QString::fromStdString(name_));
      number.append(number_);
    }
    is.close();

    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    table = new QTableWidget(f.size(),5);
    layout->addWidget(table);
    QStringList labels;
    labels << "Mode" << "Frequency" << "Exponential decay" << "Angular frequency" << "Damping ratio";
    table->setHorizontalHeaderLabels(labels);
    int n = name.size();
    QVector<double> m(name.size());
    QVector<QVector<double>> A(f.size(),QVector<double>(n));
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
    table->resizeColumnsToContents();
    if(f.size()) {
      table->selectRow(0);
      plot = new DataPlot(m,A,"Mode", "Eigenmode", "DOF", "-", this);
      layout->addWidget(plot);
      plot->setSymbol(QwtSymbol::Diamond,10);
      plot->setAxisScale(QwtPlot::xBottom,1-0.1,n+0.1,1);
      plot->setAxisScale(QwtPlot::yLeft,-1.1,1.1);
      plot->replot();
      QTreeWidget *stateTable = new QTreeWidget;
      layout->addWidget(stateTable);
      stateTable->setHeaderLabels(QStringList{"DOF","Element name","Element DOF number"});
      for(unsigned int i=0; i<name.size(); i++) {
        auto *item = new QTreeWidgetItem;
        item->setText(0, QString::number(m[i]));
        item->setText(1, name[i]);
        item->setText(2, QString::number(number[i]+1));
        stateTable->addTopLevelItem(item);
      }
      stateTable->resizeColumnToContents(1);
      QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
      layout->addWidget(buttonBox);
      buttonBox->addButton(QDialogButtonBox::Ok);
      connect(buttonBox, &QDialogButtonBox::accepted, this, &EigenanalysisDialog::accept);
      connect(plot, &DataPlot::numChanged, this, &EigenanalysisDialog::selectRow);
      connect(table, &QTableWidget::cellClicked, this, &EigenanalysisDialog::selectMode);
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

  HarmonicResponseDialog::HarmonicResponseDialog(QWidget *parent) : QDialog(parent) {
    setWindowTitle("Harmonic response analysis");
    OctaveParser parser(mw->getUniqueTempDir().generic_string()+"/harmonic_response_analysis.mat");
    parser.parse();
    fmatvec::MatV t_ = static_cast<const OctaveMatrix*>(parser.get(1))->get<fmatvec::MatV>();
    fmatvec::MatV A_ = static_cast<const OctaveMatrix*>(parser.get(2))->get<fmatvec::MatV>();

    ifstream is(mw->getUniqueTempDir().generic_string()+"/statetable.asc");
    QVector<QString> name;
    QVector<int> number;
    string name_;
    char label_;
    int number_;
    while(is) {
      is >> name_ >> label_ >> number_;
      if(label_!='q')
        break;
      name.append(QString::fromStdString(name_));
      number.append(number_);
    }
    is.close();

    QVector<double> t(t_.rows());
    QVector<QVector<double>> A(name.size(),QVector<double>(A_.rows()));
    for(int i=0; i<t_.rows(); i++) {
      t[i] = t_(i,0);
      for(int j=0; j<A.size(); j++)
        A[j][i] = A_(i,j);
    }

    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    DataPlot *plot = new DataPlot(t,A,"DOF", "Frequency response", "f in Hz", "A", this);
    layout->addWidget(plot);
    plot->replot();
    QVector<double> m(name.size());
    for(int k=0; k<name.size(); k++)
      m[k] = k+1;
    QTreeWidget *stateTable = new QTreeWidget;
    layout->addWidget(stateTable);
    stateTable->setHeaderLabels(QStringList{"DOF","Element name","Element DOF number"});
    for(unsigned int i=0; i<name.size(); i++) {
      auto *item = new QTreeWidgetItem;
      item->setText(0, QString::number(m[i]));
      item->setText(1, name[i]);
      item->setText(2, QString::number(number[i]+1));
      stateTable->addTopLevelItem(item);
    }
    stateTable->resizeColumnToContents(1);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    layout->addWidget(buttonBox);
    buttonBox->addButton(QDialogButtonBox::Ok);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &HarmonicResponseDialog::accept);
  }

  SourceDialog::SourceDialog(xercesc::DOMElement *ele, QWidget *parent) : QDialog(parent) {
    setWindowTitle(QString("XML view"));
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    XMLEditorWidget *edit = new XMLEditorWidget;
    edit->initializeUsingXML(ele);
    layout->addWidget(edit);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Ok);
    layout->addWidget(buttonBox);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &SourceDialog::accept);
  }

  StateTableDialog::StateTableDialog(QWidget *parent) : QDialog(parent) {
    setWindowTitle(QString("State table"));
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    ifstream is(mw->getUniqueTempDir().generic_string()+"/statetable.asc");
    QVector<QString> name;
    QVector<QString> label;
    QVector<QString> number;
    string name_;
    string label_;
    string number_;
    while(is) {
      is >> name_ >> label_ >> number_;
      if(not number_.empty()) {
        name.append(QString::fromStdString(name_));
        label.append(QString::fromStdString(label_));
        number.append(QString::fromStdString(number_));
      }
    }
    is.close();
    QTreeWidget *stateTable = new QTreeWidget;
    layout->addWidget(stateTable);
    stateTable->setHeaderLabels(QStringList{"State number","Name","Label","Label number"});
    for(unsigned int i=0; i<name.size(); i++) {
      auto *item = new QTreeWidgetItem;
      item->setText(0, QString::number(i));
      item->setText(1, name[i]);
      item->setText(2, label[i]);
      item->setText(3, number[i]);
      stateTable->addTopLevelItem(item);
    }
    stateTable->resizeColumnToContents(1);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Ok);
    layout->addWidget(buttonBox);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &SourceDialog::accept);
  }

}
