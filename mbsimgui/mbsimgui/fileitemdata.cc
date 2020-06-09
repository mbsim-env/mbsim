/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2020 Martin Förg

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
#include "fileitemdata.h"
#include "mainwindow.h"
#include <QDir>

namespace MBSimGUI {

  extern MainWindow *mw;

  FileItemData::FileItemData(xercesc::DOMDocument *doc_) : doc(doc_), fileInfo(QUrl(QString::fromStdString(MBXMLUtils::X()%doc->getDocumentURI())).toLocalFile()) {
    name = mw->getProjectDir().relativeFilePath(fileInfo.absoluteFilePath());
    type = QString::fromStdString(MBXMLUtils::E(getXMLElement())->getTagName().second);
  }

  void FileItemData::removeReference(EmbedItemData *item) {
    for(auto it = ref.begin(); it != ref.end(); ++it) {
      if(*it==item) {
        ref.erase(it);
        break;
      }
    }
    if(ref.empty()) mw->removeFile(this);
  }
}
