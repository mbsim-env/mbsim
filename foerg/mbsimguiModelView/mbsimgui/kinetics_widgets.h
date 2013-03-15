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

#ifndef _KINETICS_WIDGETS_H_
#define _KINETICS_WIDGETS_H_

#include "widget.h"
#include "basic_widgets.h"
#include <QComboBox>

class Function2Widget;
class QVBoxLayout;
class ExtWidget;
class ExtPhysicalVarWidget;
class Function1ChoiceWidget;
class Element;

class GeneralizedForceLawWidget : public Widget {

  public:
    GeneralizedForceLawWidget() : forceFunc(0) {}
    virtual QString getType() const { return "GeneralizedForceLaw"; }
    void updateWidget() {}
    void resizeVariables() {}
   protected:
    Function2Widget *forceFunc;
};

class BilateralConstraintWidget : public GeneralizedForceLawWidget {

  public:
    BilateralConstraintWidget() {}
    virtual QString getType() const { return "BilateralConstraint"; }
};

class RegularizedBilateralConstraintWidget : public GeneralizedForceLawWidget {
  Q_OBJECT

  friend class RegularizedBilateralConstraintProperty;

  public:
    RegularizedBilateralConstraintWidget(); 
    virtual QString getType() const { return "RegularizedBilateralConstraint"; }
  protected:
    QVBoxLayout *layout;
    QComboBox *funcList;
  protected slots:
    void defineFunction(int);
};

class UnilateralConstraintWidget : public GeneralizedForceLawWidget {

  public:
    UnilateralConstraintWidget() {}
    virtual QString getType() const { return "UnilateralConstraint"; }
};

class RegularizedUnilateralConstraintWidget : public GeneralizedForceLawWidget {
  Q_OBJECT

  friend class RegularizedUnilateralConstraintProperty;

  public:
    RegularizedUnilateralConstraintWidget(); 
    virtual QString getType() const { return "RegularizedUnilateralConstraint"; }
  protected:
    QVBoxLayout *layout;
    QComboBox *funcList;
  protected slots:
    void defineFunction(int);
};

class GeneralizedImpactLawWidget : public Widget {

  public:
    GeneralizedImpactLawWidget() {}
    virtual QString getType() const { return "GeneralizedImpactLaw"; }
    void updateWidget() {}
    void resizeVariables() {}
};

class BilateralImpactWidget : public GeneralizedImpactLawWidget {

  public:
    BilateralImpactWidget() {}
    virtual QString getType() const { return "BilateralImpact"; }
};

class UnilateralNewtonImpactWidget : public GeneralizedImpactLawWidget {

  friend class UnilateralNewtonImpactProperty;

  public:
    UnilateralNewtonImpactWidget();
    virtual QString getType() const { return "UnilateralNewtonImpact"; }
  protected:
    ExtWidget* restitutionCoefficient;
};

class FrictionForceLawWidget : public Widget {

  public:
    FrictionForceLawWidget() : frictionForceFunc(0) {}
    virtual QString getType() const { return "FrictionForceLaw"; }
    void updateWidget() {}
    void resizeVariables() {}
   protected:
    Function2Widget *frictionForceFunc;
};

class PlanarCoulombFrictionWidget : public FrictionForceLawWidget {

  friend class PlanarCoulombFrictionProperty;

  public:
    PlanarCoulombFrictionWidget();
    virtual QString getType() const { return "PlanarCoulombFriction"; }
  protected:
    ExtWidget* frictionCoefficient;
};

class SpatialCoulombFrictionWidget : public FrictionForceLawWidget {

  friend class SpatialCoulombFrictionProperty;

  public:
    SpatialCoulombFrictionWidget();
    virtual QString getType() const { return "SpatialCoulombFriction"; }
  protected:
    ExtWidget* frictionCoefficient;
};

class RegularizedPlanarFrictionWidget : public FrictionForceLawWidget {
  Q_OBJECT

  friend class RegularizedPlanarFrictionProperty;

  public:
    RegularizedPlanarFrictionWidget(); 
    virtual QString getType() const { return "RegularizedPlanarFriction"; }
  protected:
    QVBoxLayout *layout;
    QComboBox *funcList;
  protected slots:
    void defineFunction(int);
};

class RegularizedSpatialFrictionWidget : public FrictionForceLawWidget {
  Q_OBJECT

  friend class RegularizedSpatialFrictionProperty;

  public:
    RegularizedSpatialFrictionWidget(); 
    virtual QString getType() const { return "RegularizedSpatialFriction"; }
  protected:
    QVBoxLayout *layout;
    QComboBox *funcList;
  protected slots:
    void defineFunction(int);
};

class FrictionImpactLawWidget : public Widget {

  public:
    FrictionImpactLawWidget() {}
    virtual QString getType() const { return "FrictionImpactLaw"; }
    void updateWidget() {}
    void resizeVariables() {}
};

class PlanarCoulombImpactWidget : public FrictionImpactLawWidget {

  friend class PlanarCoulombImpactProperty;

  public:
    PlanarCoulombImpactWidget();
    virtual QString getType() const { return "PlanarCoulombImpact"; }
  protected:
    ExtWidget* frictionCoefficient;
};

class SpatialCoulombImpactWidget : public FrictionImpactLawWidget {

  friend class SpatialCoulombImpactProperty;

  public:
    SpatialCoulombImpactWidget();
    virtual QString getType() const { return "SpatialCoulombImpact"; }
  protected:
    ExtWidget* frictionCoefficient;
};

class GeneralizedForceLawChoiceWidget : public Widget {
  Q_OBJECT

  friend class GeneralizedForceLawChoiceProperty;

  public:
    GeneralizedForceLawChoiceWidget();

    int getForceLaw() {return comboBox->currentIndex();}
    void updateWidget() {}
    void resizeVariables() {}

  protected slots:
    void defineForceLaw(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    GeneralizedForceLawWidget *generalizedForceLaw;
};

class GeneralizedImpactLawChoiceWidget : public Widget {
  Q_OBJECT

  friend class GeneralizedImpactLawChoiceProperty;

  public:
    GeneralizedImpactLawChoiceWidget();

    int getImpactLaw() {return comboBox->currentIndex();}
    void updateWidget() {}
    void resizeVariables() {}

  protected slots:
    void defineImpactLaw(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    GeneralizedImpactLawWidget *generalizedImpactLaw;
};

class FrictionForceLawChoiceWidget : public Widget {
  Q_OBJECT

  friend class FrictionForceLawChoiceProperty;

  public:
    FrictionForceLawChoiceWidget();

    int getForceLaw() {return comboBox->currentIndex();}
    void updateWidget() {}
    void resizeVariables() {}

  protected slots:
    void defineFrictionLaw(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    FrictionForceLawWidget *frictionForceLaw;
};

class FrictionImpactLawChoiceWidget : public Widget {
  Q_OBJECT

  friend class FrictionImpactLawChoiceProperty;

  public:
    FrictionImpactLawChoiceWidget();

    int getImpactLaw() {return comboBox->currentIndex();}
    void updateWidget() {}
    void resizeVariables() {}

  protected slots:
    void defineFrictionImpactLaw(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    FrictionImpactLawWidget *frictionImpactLaw;
};

class GeneralizedForceChoiceWidget : public Widget {

  friend class GeneralizedForceChoiceProperty;

  public:
    GeneralizedForceChoiceWidget();

    int getSize() const; 
    void updateWidget() {}
    void resizeVariables() {}

  protected:
    QVBoxLayout *layout;
    ExtWidget *generalizedForceLaw, *generalizedImpactLaw, *mat;
};

class ForceChoiceWidget : public Widget {
  Q_OBJECT

  friend class ForceChoiceProperty;

  public:
    ForceChoiceWidget();

    int getSize() const; 
    void updateWidget() {}

  public slots:
    void resizeVariables();

  protected:
    QVBoxLayout *layout;
    ExtWidget *forceLaw, *mat;
};

class ForceDirectionWidget : public Widget {

  friend class ForceDirectionProperty;

  public:
    ForceDirectionWidget(Element *element);

    void updateWidget() {}
    void resizeVariables() {}

  protected:
    QWidget *forceDirWidget;
    Element *element;
    ExtWidget *refFrame, *mat;
};

class GeneralizedForceDirectionWidget : public Widget {

  friend class GeneralizedForceDirectionProperty;

  public:
    GeneralizedForceDirectionWidget();

    int getSize() const;
    void updateWidget() {}
    void resizeVariables() {}

  protected:
    ExtWidget *mat;
};

#endif

