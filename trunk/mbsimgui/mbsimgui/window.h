#ifndef _WINDOW_H_
#define _WINDOW_H_

#include <QWidget>
#include <QDoubleSpinBox>

class Window: public QWidget {
  private:
    QDoubleSpinBox *box[3];
  public:
    Window(QWidget *parent = 0);
    QDoubleSpinBox* getBox(int j) {return box[j];}
};

#endif
