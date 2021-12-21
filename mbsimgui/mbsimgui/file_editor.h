/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2017 Martin Förg

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

#ifndef __FILE_EDITOR_H_
#define __FILE_EDITOR_H_

#include <QDialog>

class QTextEdit;

namespace MBSimGUI {

  class FileEditor : public QDialog {
    public:
      FileEditor(const QString &title, const QString &filename, int n, const QString &text, QWidget *parent);
    private:
      QTextEdit *edit;
  };

}

#endif
