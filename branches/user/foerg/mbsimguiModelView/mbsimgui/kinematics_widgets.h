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

#ifndef _KINEMATICS_WIDGETS_H_
#define _KINEMATICS_WIDGETS_H_

#include "widget.h"
#include <QComboBox>

class ExtPhysicalVarWidget;
class ExtWidget;
class QVBoxLayout;

class TranslationWidget : public Widget {

  public:
    TranslationWidget() {}
    virtual int getSize() const = 0;
    void updateWidget() {}
    void resizeVariables() {}
   protected:
};

class LinearTranslationWidget : public TranslationWidget {
  Q_OBJECT

  friend class LinearTranslationProperty;

  public:
    LinearTranslationWidget();
    int getSize() const;
  protected:
    ExtWidget *mat;
  signals:
    void translationChanged();
};

class TranslationChoiceWidget : public Widget {
  Q_OBJECT

  friend class TranslationChoiceProperty;

  public:
    TranslationChoiceWidget(const std::string &xmlName);

    int getSize() const { return translation->getSize(); }
    void updateWidget() {}
    void resizeVariables() {}

  protected slots:
    void defineTranslation(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    TranslationWidget *translation;
    std::string xmlName;
  signals:
    void translationChanged();
};

class RotationWidget : public Widget {

  public:
    RotationWidget() {}
    virtual int getSize() const = 0;
    void updateWidget() {}
    void resizeVariables() {}
};

class RotationAboutXAxisWidget : public RotationWidget {

  public:
    RotationAboutXAxisWidget() {}
    virtual int getSize() const {return 1;}
};

class RotationAboutYAxisWidget : public RotationWidget {

  public:
    RotationAboutYAxisWidget() {}
    virtual int getSize() const {return 1;}
};

class RotationAboutZAxisWidget : public RotationWidget {

  public:
    RotationAboutZAxisWidget() {}
    virtual int getSize() const {return 1;}
};

class RotationAboutFixedAxisWidget : public RotationWidget {

  friend class RotationAboutFixedAxisProperty;

  public:
    RotationAboutFixedAxisWidget();
    virtual int getSize() const {return 1;}
  protected:
    ExtWidget *vec;
};

class RotationAboutAxesXYWidget : public RotationWidget {

  public:
    RotationAboutAxesXYWidget() {}
    virtual int getSize() const {return 2;}
};

class CardanAnglesWidget : public RotationWidget {

  public:
    CardanAnglesWidget() {}
    virtual int getSize() const {return 3;}
};

class RotationChoiceWidget : public Widget {
  Q_OBJECT

  friend class RotationChoiceProperty;

  public:
    RotationChoiceWidget(const std::string &xmlName);

    int getSize() const { return rotation->getSize(); }
    void updateWidget() {}
    void resizeVariables() {}

  public slots:
    void defineRotation(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    RotationWidget *rotation;
    std::string xmlName;
  signals:
    void rotationChanged();
};

#endif

