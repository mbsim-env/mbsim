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

#ifndef _FILEITEMDATA_H
#define _FILEITEMDATA_H

#include "treeitemdata.h"
#include "embeditemdata.h"
#include <QFileInfo>
#include <QUrl>

namespace MBSimGUI {

  class EmbedItemData;

  class FileItemData : public TreeItemData {
    public:

      FileItemData(xercesc::DOMDocument *doc_) : doc(doc_), fileInfo(QUrl(QString::fromStdString(MBXMLUtils::X()%doc->getDocumentURI())).toLocalFile()) { }

      QString getName() const override { return fileInfo.fileName()+(modified?"*":""); }
      QString getType() const override { return ""; }
      QString getValue() const override { return fileInfo.canonicalFilePath(); }

      const QFileInfo& getFileInfo() const { return fileInfo; }
      xercesc::DOMDocument *getXMLDocument() { return doc; }
      xercesc::DOMElement *getXMLElement() { return doc->getDocumentElement(); }

      void addReference(EmbedItemData *item) { ref.push_back(item); }
      int getNumberOfReferences() const { return ref.size(); }
      EmbedItemData *getReference(int i) { return ref[i]; }
      void setModified(bool modified_) { modified = modified_; }
      bool getModified() const { return modified; }

    protected:
      xercesc::DOMDocument *doc;
      QFileInfo fileInfo;
      std::vector<EmbedItemData*> ref;
      bool modified{false};
  };

}

#endif
