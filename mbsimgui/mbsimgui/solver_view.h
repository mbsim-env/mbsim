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

#ifndef _SOLVER_VIEW__H_
#define _SOLVER_VIEW__H_

#include <QMenu>
#include <QLineEdit>

namespace MBSimGUI {

  class Solver;

  class SolverContextMenu : public QMenu {
    public:
      SolverContextMenu(const std::vector<QString> &type, QWidget *parent=nullptr);
    private:
      void selectSolver(QAction *action);
  };

  class SolverView : public QWidget {
    public:
      SolverView();
      ~SolverView() override = default;
      void setText(const QString &text) { name->setText(text); }
      QString text()const { return name->text(); }
      bool hasFocus() const { return name->hasFocus(); }
      void setSolver(int i_) { i = i_; setText(type[i]); file->setText(""); }
      void setSolver(Solver *solver);
      Solver* createSolver(int i_);
      QMenu* createContextMenu() { return new SolverContextMenu(type,this); }
    private:
      QLineEdit *name;
      QLineEdit *file;
      std::vector<QString> type;
      int i{0};
  };

  class SolverMouseEvent : public QObject {
    public:
      SolverMouseEvent(QLineEdit *lineEdit) : QObject(lineEdit) { }
    private:
      bool eventFilter(QObject *obj, QEvent *event) override;
  };

}

#endif
