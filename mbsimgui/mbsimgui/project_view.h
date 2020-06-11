/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2017 Martin Förg

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

#ifndef _PROJECT_VIEW__H_
#define _PROJECT_VIEW__H_

#include <QLineEdit>
#include <QMenu>

namespace MBSimGUI {

  class Project;

  class ProjectContextMenu : public QMenu {
    public:
      ProjectContextMenu(QWidget * parent = nullptr);
  };

  class ProjectView : public QWidget {
    public:
      ProjectView();
      ~ProjectView() override = default;
      void setText(const QString &text) { name->setText(text); }
      QString text()const { return name->text(); }
      bool hasFocus() const { return name->hasFocus(); }
      QMenu* createContextMenu() { return new ProjectContextMenu(this); }
    private:
      QLineEdit *name;
  };

  class ProjectMouseEvent : public QObject {
    public:
      ProjectMouseEvent(QLineEdit* lineEdit) : QObject(lineEdit) { }
    protected:
      bool eventFilter(QObject *obj, QEvent *event) override;
  };

}

#endif
