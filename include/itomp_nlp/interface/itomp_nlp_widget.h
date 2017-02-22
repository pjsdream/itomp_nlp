#ifndef ITOMP_INTERFACE_ITOMP_NLP_WIDGET_H
#define ITOMP_INTERFACE_ITOMP_NLP_WIDGET_H


#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSignalMapper>

#include <itomp_nlp/optimization/optimizer.h>


namespace itomp
{

class ItompNLPWidget : public QWidget
{
    Q_OBJECT

public:

    ItompNLPWidget(QWidget* parent = 0);

private:

};

}


#endif // ITOMP_INTERFACE_ITOMP_NLP_WIDGET_H