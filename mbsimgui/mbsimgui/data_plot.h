/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2017 Martin Förg

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

#ifndef _DATA_PLOT__H_
#define _DATA_PLOT__H_

#include <QWidget>
#include <qwt_symbol.h>

class QwtPlot;
class QwtPlotCurve;
class QSpinBox;

namespace MBSimGUI {

  class DataPlot : public QWidget {
    Q_OBJECT
    public:
      DataPlot(const QVector<double> &f_, const QVector<QVector<double> > &A_, const QString &spinBoxLabel="", const QString &title="", const QString &xLabel="", const QString &yLabel="",  QWidget *parent=nullptr);
      void setSymbol(const QwtSymbol::Style &style, int size=1);
      void setAxisScale(int axisId, double min, double max, double stepSize = 0);
      void replot();
    private:
      QVector<double> f;
      QVector<QVector<double> > A;
      QSpinBox *num;
      QwtPlot *plot;
      QwtPlotCurve *curve;
    public slots:
      void changePlot(int i);
      void changeNum(int i);
    signals:
      void numChanged(int i);
  };

}

#endif
