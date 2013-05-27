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
   protected:
};

class TranslationInXDirectionWidget : public TranslationWidget {

  public:
    TranslationInXDirectionWidget() {}
    virtual int getSize() const {return 1;}
};

class TranslationInYDirectionWidget : public TranslationWidget {

  public:
    TranslationInYDirectionWidget() {}
    virtual int getSize() const {return 1;}
};

class TranslationInZDirectionWidget : public TranslationWidget {

  public:
    TranslationInZDirectionWidget() {}
    virtual int getSize() const {return 1;}
};

class TranslationInXYDirectionWidget : public TranslationWidget {

  public:
    TranslationInXYDirectionWidget() {}
    virtual int getSize() const {return 2;}
};

class TranslationInXZDirectionWidget : public TranslationWidget {

  public:
    TranslationInXZDirectionWidget() {}
    virtual int getSize() const {return 2;}
};

class TranslationInYZDirectionWidget : public TranslationWidget {

  public:
    TranslationInYZDirectionWidget() {}
    virtual int getSize() const {return 2;}
};

class TranslationInXYZDirectionWidget : public TranslationWidget {

  public:
    TranslationInXYZDirectionWidget() {}
    virtual int getSize() const {return 3;}
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

class TimeDependentTranslationWidget : public TranslationWidget {

  friend class TimeDependentTranslationProperty;

  public:
    TimeDependentTranslationWidget();
    int getSize() const {return 0;}
  protected:
    ExtWidget *function;
};

class StateDependentTranslationWidget : public TranslationWidget {

  friend class StateDependentTranslationProperty;

  public:
    StateDependentTranslationWidget();
    int getSize() const {return 0;}
  protected:
    ExtWidget *function;
};

class GeneralTranslationWidget : public TranslationWidget {

  friend class GeneralTranslationProperty;

  public:
    GeneralTranslationWidget();
    int getSize() const {return 0;}
  protected:
    ExtWidget *function;
};

class TranslationChoiceWidget : public Widget {
  Q_OBJECT

  friend class TranslationChoiceProperty;

  public:
    TranslationChoiceWidget();

    int getSize() const { return translation->getSize(); }

  protected slots:
    void defineTranslation(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    TranslationWidget *translation;
  signals:
    void translationChanged();
};

class RotationWidget : public Widget {

  public:
    RotationWidget() {}
    virtual int getSize() const = 0;
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

class RotationAboutAxesXZWidget : public RotationWidget {

  public:
    RotationAboutAxesXZWidget() {}
    virtual int getSize() const {return 2;}
};

class RotationAboutAxesYZWidget : public RotationWidget {

  public:
    RotationAboutAxesYZWidget() {}
    virtual int getSize() const {return 2;}
};

class RotationAboutAxesXYZWidget : public RotationWidget {

  public:
    RotationAboutAxesXYZWidget() {}
    virtual int getSize() const {return 3;}
};

class CardanAnglesWidget : public RotationWidget {

  public:
    CardanAnglesWidget() {}
    virtual int getSize() const {return 3;}
};

class EulerAnglesWidget : public RotationWidget {

  public:
    EulerAnglesWidget() {}
    virtual int getSize() const {return 3;}
};

class TimeDependentRotationAboutFixedAxisWidget : public RotationWidget {

  friend class TimeDependentRotationAboutFixedAxisProperty;

  public:
    TimeDependentRotationAboutFixedAxisWidget();
    int getSize() const {return 0;}
  protected:
    ExtWidget *vec, *function;
};

class StateDependentRotationAboutFixedAxisWidget : public RotationWidget {

  friend class StateDependentRotationAboutFixedAxisProperty;

  public:
    StateDependentRotationAboutFixedAxisWidget();
    int getSize() const {return 0;}
  protected:
    ExtWidget *vec, *function;
};

class RotationChoiceWidget : public Widget {
  Q_OBJECT

  friend class RotationChoiceProperty;

  public:
    RotationChoiceWidget();

    int getSize() const { return rotation->getSize(); }

  public slots:
    void defineRotation(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    RotationWidget *rotation;
  signals:
    void rotationChanged();
};

#endif

