/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2016 Martin Förg

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

#ifndef _IMPORT_WIDGETS_H_
#define _IMPORT_WIDGETS_H_

#include "widget.h"
#include "basic_widgets.h"
#include <QCheckBox>

namespace MBSimGUI {

  class FileWidget;

  class ImportWidget : public Widget {
    Q_OBJECT
    public:
      ImportWidget();

      bool getMassChecked() { return labelMass->isChecked(); }
      bool getrdmChecked() { return labelrdm->isChecked(); }
      bool getrrdmChecked() { return labelrrdm->isChecked(); }
      bool getPdmChecked() { return labelPdm->isChecked(); }
      bool getrPdmChecked() { return labelrPdm->isChecked(); }
      bool getPPdmChecked() { return labelPPdm->isChecked(); }
      bool getKeChecked() { return labelKe->isChecked(); }
      bool getu0Checked() { return labelu0->isChecked(); }
      bool getPhiChecked() { return labelPhi->isChecked(); }
      bool getSrChecked() { return labelSr->isChecked(); }
      bool getNodesChecked() { return labelNodes->isChecked(); }
      bool getIndicesChecked() { return labelIndices->isChecked(); }
      bool getNumberOfModesChecked() { return labelnm->isChecked(); }

      bool getFilerrdmChecked() { return checkrrdm->isChecked(); }
      bool getFilePdmChecked() { return checkPdm->isChecked(); }
      bool getFilerPdmChecked() { return checkrPdm->isChecked(); }
      bool getFilePPdmChecked() { return checkPPdm->isChecked(); }
      bool getFileKeChecked() { return checkKe->isChecked(); }
      bool getFileu0Checked() { return checku0->isChecked(); }
      bool getFilePhiChecked() { return checkPhi->isChecked(); }
      bool getFileSrChecked() { return checkSr->isChecked(); }
      bool getFileNodesChecked() { return checkNodes->isChecked(); }
      bool getFileIndicesChecked() { return checkIndices->isChecked(); }
      int getNumberOfModes() { return nm->value(); }

      QString getFilenamerdm() { return filerdm->getFile(); }
      QString getFilenamerrdm() { return filerrdm->getFile(); }
      QString getFilenamePdm() { return filePdm->getFile(); }
      QString getFilenamerPdm() { return filerPdm->getFile(); }
      QString getFilenamePPdm() { return filePPdm->getFile(); }
      QString getFilenameKe() { return fileKe->getFile(); }
      QString getFilenameu0() { return fileu0->getFile(); }
      QString getFilenamePhi() { return filePhi->getFile(); }
      QString getFilenameSr() { return fileSr->getFile(); }
      QString getFilenameNodes() { return fileNodes->getFile(); }
      QString getFilenameIndices() { return fileIndices->getFile(); }
      QString getResultFile() { return resultFile->getFile(); }

    private:
      CustomSpinBox *nm;
      QCheckBox *labelnm, *labelMass, *labelrdm, *labelrrdm, *labelPdm, *labelrPdm, *labelPPdm, *labelKe, *labelu0, *labelPhi, *labelSr, *labelNodes, *labelIndices, *checkrdm, *checkrrdm, *checkPdm, *checkrPdm, *checkPPdm, *checkKe, *checku0, *checkPhi, *checkSr, *checkNodes, *checkIndices;
      FileWidget *filerdm, *filerrdm, *filePdm, *filerPdm, *filePPdm, *fileKe, *fileu0, *filePhi, *fileSr, *fileNodes, *fileIndices, *resultFile;
    private slots:
      void updatePdm();
      void updaterPdm();
      void updatePPdm();
      void updateKe();
      void updateu0();
      void updatePhi();
      void updateSr();
      void updateNodes();
      void updateIndices();
  };

}

#endif
