/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2020 Martin Förg

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
#include "fileitemdata.h"
#include "mainwindow.h"
#include <QDir>

namespace MBSimGUI {

  extern MainWindow *mw;

  FileItemData::FileItemData(const std::shared_ptr<xercesc::DOMDocument> &doc_) : doc(doc_) {
    auto docFilename = MBXMLUtils::D(doc)->getDocumentFilename();
    fileInfo = QFileInfo(docFilename.string().c_str());
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
