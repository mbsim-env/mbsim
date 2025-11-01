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

      FileItemData(const std::shared_ptr<xercesc::DOMDocument> &doc_);

      QString getName() const override { return name+(modified?"*":""); }
      QString getType() const override { return type; }
      QString getValue() const override { return QString::number(ref.size()); }

      const QFileInfo& getFileInfo() const { return fileInfo; }
      std::shared_ptr<xercesc::DOMDocument> getXMLDocument() { return doc; }
      xercesc::DOMElement *getXMLElement() override { return doc->getDocumentElement(); }

      void addReference(EmbedItemData *item) { ref.push_back(item); }
      void removeReference(EmbedItemData *item);
      int getNumberOfReferences() const { return ref.size(); }
      EmbedItemData *getFileReference(int i) { return ref[i]; }
      void setModified(bool modified_) { modified = modified_; }
      bool getModified() const { return modified; }

    protected:
      std::shared_ptr<xercesc::DOMDocument> doc;
      QFileInfo fileInfo;
      std::vector<EmbedItemData*> ref;
      bool modified{false};
      QString name, type;
  };

}

#endif
