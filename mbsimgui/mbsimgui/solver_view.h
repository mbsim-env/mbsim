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

#include <QLineEdit>
#include <QMenu>

namespace MBSimGUI {

  class Solver;
  class SolverPropertyDialog;

  class SolverViewContextMenu : public QMenu {
    Q_OBJECT

    public:
      SolverViewContextMenu(QWidget * parent = nullptr);
    protected slots:
      void selectSolver(QAction *action);
  };

  class SolverView : public QLineEdit {
    Q_OBJECT
    public:
      SolverView();
      ~SolverView() override = default;
      int getSolverNumber() const { return i; }
      void setSolver(int i_) { i = i_; updateText(); }
      void setSolver(Solver *solver);
      Solver* createSolver(int i_);
      void updateText() { setText(type[i]); }
      QMenu* createContextMenu() { return new SolverViewContextMenu; }
    protected:
      std::vector<QString> type;
      int i{0};
    protected slots:
      void openContextMenu();
  };

  class SolverMouseEvent : public QObject {
    Q_OBJECT
    public:
      SolverMouseEvent(SolverView* view_) : view(view_) { }
    protected:
      SolverView *view;
      SolverPropertyDialog *editor;
      bool eventFilter(QObject *obj, QEvent *event) override;
    protected slots:
      void dialogFinished(int result);
      void apply();
  };

}

#endif
