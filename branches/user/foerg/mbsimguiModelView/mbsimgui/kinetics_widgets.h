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

class Function2;
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
    Function2 *forceFunc;
};

class BilateralConstraint : public GeneralizedForceLawWidget {

  public:
    BilateralConstraint() {}
    virtual QString getType() const { return "BilateralConstraint"; }
};

class RegularizedBilateralConstraint : public GeneralizedForceLawWidget {
  Q_OBJECT

  public:
    RegularizedBilateralConstraint(); 
    virtual QString getType() const { return "RegularizedBilateralConstraint"; }
  protected:
    QVBoxLayout *layout;
    QComboBox *funcList;
  protected slots:
    void defineFunction(int);
};

class UnilateralConstraint : public GeneralizedForceLawWidget {

  public:
    UnilateralConstraint() {}
    virtual QString getType() const { return "UnilateralConstraint"; }
};

class RegularizedUnilateralConstraint : public GeneralizedForceLawWidget {
  Q_OBJECT

  public:
    RegularizedUnilateralConstraint(); 
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

class BilateralImpact : public GeneralizedImpactLawWidget {

  public:
    BilateralImpact() {}
    virtual QString getType() const { return "BilateralImpact"; }
};

class UnilateralNewtonImpact : public GeneralizedImpactLawWidget {

  public:
    UnilateralNewtonImpact();
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
    Function2 *frictionForceFunc;
};

class PlanarCoulombFriction : public FrictionForceLawWidget {

  public:
    PlanarCoulombFriction();
    virtual QString getType() const { return "PlanarCoulombFriction"; }
  protected:
    ExtWidget* frictionCoefficient;
};

class SpatialCoulombFriction : public FrictionForceLawWidget {

  public:
    SpatialCoulombFriction();
    virtual QString getType() const { return "SpatialCoulombFriction"; }
  protected:
    ExtWidget* frictionCoefficient;
};

class RegularizedPlanarFriction : public FrictionForceLawWidget {
  Q_OBJECT

  public:
    RegularizedPlanarFriction(); 
    virtual QString getType() const { return "RegularizedPlanarFriction"; }
  protected:
    QVBoxLayout *layout;
    QComboBox *funcList;
  protected slots:
    void defineFunction(int);
};

class RegularizedSpatialFriction : public FrictionForceLawWidget {
  Q_OBJECT

  public:
    RegularizedSpatialFriction(); 
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

class PlanarCoulombImpact : public FrictionImpactLawWidget {

  public:
    PlanarCoulombImpact();
    virtual QString getType() const { return "PlanarCoulombImpact"; }
  protected:
    ExtWidget* frictionCoefficient;
};

class SpatialCoulombImpact : public FrictionImpactLawWidget {

  public:
    SpatialCoulombImpact();
    virtual QString getType() const { return "SpatialCoulombImpact"; }
  protected:
    ExtWidget* frictionCoefficient;
};

class GeneralizedForceLawChoiceWidget : public Widget {
  Q_OBJECT

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

  public:
    GeneralizedForceChoiceWidget(ExtWidget* arrow);

    int getSize() const; 
    void updateWidget() {}
    void resizeVariables() {}

  protected:
    QVBoxLayout *layout;
    GeneralizedForceLawChoiceWidget *generalizedForceLaw_;
    GeneralizedImpactLawChoiceWidget *generalizedImpactLaw_;
    ExtPhysicalVarWidget *mat_;
    ExtWidget *generalizedForceLaw, *generalizedImpactLaw, *mat;
    ExtWidget *arrow;
};

class ForceChoiceWidget : public Widget {
  Q_OBJECT

  public:
    ForceChoiceWidget(ExtWidget* arrow);

    int getSize() const; 
    void updateWidget() {}

  public slots:
    void resizeVariables();

  protected:
    QVBoxLayout *layout;
    ExtPhysicalVarWidget *widget;
    ExtWidget *arrow;
    Function1ChoiceWidget* forceLaw;
};

class ForceDirectionWidget : public Widget {

  public:
    ForceDirectionWidget(Element *element);

    void updateWidget() {}
    void resizeVariables() {}

  protected:
    QWidget *forceDirWidget;
    FrameOfReferenceWidget* refFrame;
    Element *element;
    ExtPhysicalVarWidget *mat;
};

class GeneralizedForceDirectionWidget : public Widget {

  public:
    GeneralizedForceDirectionWidget();

    int getSize() const;
    void updateWidget() {}
    void resizeVariables() {}

  protected:
    ExtPhysicalVarWidget *mat;
};

#endif

