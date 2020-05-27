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
#include <xercesc/util/XercesDefs.hpp>

namespace XERCES_CPP_NAMESPACE {
  class DOMDocument;
  class DOMElement;
  class DOMNode;
}

namespace MBSimGUI {

  class EmbedItemData;

  class FileItemData : public TreeItemData {
    public:

      FileItemData(const QFileInfo &fileInfo_, xercesc::DOMDocument *doc_) : fileInfo(fileInfo_), doc(doc_) { } 

      QString getName() const override { return fileInfo.absoluteFilePath(); }
      QString getType() const override { return ""; }
      QString getValue() const override { return ""; }

      const QFileInfo& getFileInfo() const { return fileInfo; }
      EmbedItemData* getItem() { return item; }
      xercesc::DOMDocument *getXMLDocument() { return doc; }
      xercesc::DOMElement *getXMLElement() { return doc->getDocumentElement(); }

      void setItem(EmbedItemData *item_) { item = item_; }
      void addReference(EmbedItemData *item) { ref.push_back(item); }
      int getNumberOfReferences() const { return ref.size(); }
      EmbedItemData *getReference(int i) { return ref[i]; }

      PropertyDialog* createPropertyDialog() { return item->createPropertyDialog(); }

    protected:
      QFileInfo fileInfo;
      xercesc::DOMDocument *doc;
      EmbedItemData *item{nullptr};
      std::vector<EmbedItemData*> ref;
  };

}

#endif
