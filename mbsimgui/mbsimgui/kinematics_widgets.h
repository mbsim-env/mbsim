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
class QStackedWidget;

class TranslationWidget : public Widget {

  public:
    virtual int getqSize() const {return 0;}
    virtual int getuSize() const {return getqSize();}
    virtual int getqTSize() const {return 0;}
    virtual int getuTSize() const {return getqTSize();}
};

class RotationIndependentTranslationWidget : public TranslationWidget {
};

class TranslationInXDirectionWidget : public RotationIndependentTranslationWidget {

  public:
    virtual int getqTSize() const {return 1;}
};

class TranslationInYDirectionWidget : public RotationIndependentTranslationWidget {

  public:
    virtual int getqTSize() const {return 1;}
};

class TranslationInZDirectionWidget : public RotationIndependentTranslationWidget {

  public:
    virtual int getqTSize() const {return 1;}
};

class TranslationInXYDirectionWidget : public RotationIndependentTranslationWidget {

  public:
    virtual int getqTSize() const {return 2;}
};

class TranslationInXZDirectionWidget : public RotationIndependentTranslationWidget {

  public:
    virtual int getqTSize() const {return 2;}
};

class TranslationInYZDirectionWidget : public RotationIndependentTranslationWidget {

  public:
    virtual int getqTSize() const {return 2;}
};

class TranslationInXYZDirectionWidget : public RotationIndependentTranslationWidget {

  public:
    virtual int getqTSize() const {return 3;}
};

class LinearTranslationWidget : public RotationIndependentTranslationWidget {
  Q_OBJECT

  friend class LinearTranslationProperty;

  public:
    LinearTranslationWidget();
    int getqTSize() const;
  protected:
    ExtWidget *mat;
  signals:
    void translationChanged();
};

class TimeDependentTranslationWidget : public RotationIndependentTranslationWidget {

  friend class TimeDependentTranslationProperty;

  public:
    TimeDependentTranslationWidget();
    int getqTSize() const {return 0;}
  protected:
    ExtWidget *function;
};

class StateDependentTranslationWidget : public TranslationWidget {

  friend class StateDependentTranslationProperty;

  public:
    StateDependentTranslationWidget();
    int getqSize() const;
  protected:
    ExtWidget *function;
};

class GeneralTranslationWidget : public TranslationWidget {

  friend class GeneralTranslationProperty;

  public:
    GeneralTranslationWidget();
    int getqSize() const {return 0;}
  protected:
    ExtWidget *function;
};

class TranslationChoiceWidget : public Widget {
  Q_OBJECT

  friend class TranslationChoiceProperty;

  public:
    TranslationChoiceWidget();

    int getqSize() const { return getTranslation()->getqSize(); }
    int getuSize() const { return getTranslation()->getuSize(); }
    int getqTSize() const { return getTranslation()->getqTSize(); }
    int getuTSize() const { return getTranslation()->getuTSize(); }

    bool isIndependent() const {return dynamic_cast<RotationIndependentTranslationWidget*>(getTranslation())!=NULL;}

    TranslationWidget* getTranslation(int i);
    TranslationWidget* getTranslation() const;

  protected slots:
    void defineTranslation(int);

  protected:
    QComboBox *comboBox;
    QStackedWidget *stackedWidget;
  signals:
    void translationChanged();
};

class RotationWidget : public Widget {

  public:
    virtual int getqSize() const {return 0;}
    virtual int getuSize() const {return getqSize();}
    virtual int getqRSize() const {return 0;}
    virtual int getuRSize() const {return getqRSize();}
};

class TranslationIndependentRotationWidget : public RotationWidget  {
};

class RotationAboutXAxisWidget : public TranslationIndependentRotationWidget {

  public:
    virtual int getqRSize() const {return 1;}
};

class RotationAboutYAxisWidget : public TranslationIndependentRotationWidget {

  public:
    virtual int getqRSize() const {return 1;}
};

class RotationAboutZAxisWidget : public TranslationIndependentRotationWidget {

  public:
    virtual int getqRSize() const {return 1;}
};

class RotationAboutFixedAxisWidget : public TranslationIndependentRotationWidget {

  friend class RotationAboutFixedAxisProperty;

  public:
    RotationAboutFixedAxisWidget();
    virtual int getqRSize() const {return 1;}
  protected:
    ExtWidget *vec;
};

class RotationAboutAxesXYWidget : public TranslationIndependentRotationWidget {

  public:
    virtual int getqRSize() const {return 2;}
};

class RotationAboutAxesXZWidget : public TranslationIndependentRotationWidget {

  public:
    virtual int getqRSize() const {return 2;}
};

class RotationAboutAxesYZWidget : public TranslationIndependentRotationWidget {

  public:
    virtual int getqRSize() const {return 2;}
};

class RotationAboutAxesXYZWidget : public TranslationIndependentRotationWidget {

  public:
    virtual int getqRSize() const {return 3;}
};

class CardanAnglesWidget : public TranslationIndependentRotationWidget {

  public:
    virtual int getqRSize() const {return 3;}
};

class EulerAnglesWidget : public TranslationIndependentRotationWidget {

  public:
    virtual int getqRSize() const {return 3;}
};

class TimeDependentRotationAboutFixedAxisWidget : public TranslationIndependentRotationWidget {

  friend class TimeDependentRotationAboutFixedAxisProperty;

  public:
    TimeDependentRotationAboutFixedAxisWidget();
    int getqRSize() const {return 0;}
  protected:
    ExtWidget *vec, *function;
};

class StateDependentRotationAboutFixedAxisWidget : public RotationWidget {

  friend class StateDependentRotationAboutFixedAxisProperty;

  public:
    StateDependentRotationAboutFixedAxisWidget();
    int getqSize() const;
  protected:
    ExtWidget *vec, *function;
};

class RotationChoiceWidget : public Widget {
  Q_OBJECT

  friend class RotationChoiceProperty;

  public:
    RotationChoiceWidget();

    int getqSize() const { return getRotation()->getqSize(); }
    int getuSize() const { return getRotation()->getuSize(); }
    int getqRSize() const { return getRotation()->getqRSize(); }
    int getuRSize() const { return getRotation()->getuRSize(); }

    bool isIndependent() const {return dynamic_cast<TranslationIndependentRotationWidget*>(getRotation())!=NULL;}

    RotationWidget* getRotation(int i);
    RotationWidget* getRotation() const;

  public slots:
    void defineRotation(int);

  protected:
    QComboBox *comboBox;
    QStackedWidget *stackedWidget;
  signals:
    void rotationChanged();
};

#endif

