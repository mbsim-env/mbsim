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

#include <config.h>
#include "file_editor.h"
#include <QTextEdit>
#include <QGridLayout>
#include <QFile>
#include <QTextStream>
#include <QDialogButtonBox>

namespace MBSimGUI {

  FileEditor::FileEditor(const QString &title, const QString &filename, int n, const QString &text, QWidget *parent) : QDialog(parent) {

    resize(QSize(800,400));

    auto *layout = new QVBoxLayout;
    setLayout(layout);

    setWindowTitle(title);

    edit=new QTextEdit(this);
    layout->addWidget(edit);

    auto *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Close);
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
    layout->addWidget(buttonBox);

    QFile file(filename);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
      edit->setPlainText(text);
      return;
    }
    QTextStream in(&file);
    edit->setPlainText(in.readAll());
    for(int i=0; i<n-1; i++)
      edit->moveCursor(QTextCursor::Down);
  }

}
