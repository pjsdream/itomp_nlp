#ifndef ITOMP_INTERFACE_ITOMP_COST_FUNCTION_WIDGET_H
#define ITOMP_INTERFACE_ITOMP_COST_FUNCTION_WIDGET_H


#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSignalMapper>

#include <itomp_nlp/optimization/optimizer.h>


namespace itomp
{

class ItompCostFunctionWidget : public QWidget
{
    Q_OBJECT

private:

    static const std::vector<std::pair<std::string, int> > cost_types_;

public:

    ItompCostFunctionWidget(int id, QWidget* parent = 0);

Q_SIGNALS:

    void valueChanged(int id, const std::string& cost_type, std::vector<double> coefficients);
    
public Q_SLOTS:

    void slotValueChanged();
    void selectCostType(int index);

private:

    int id_;

    QGridLayout* layout_;

    QComboBox* combo_box_;

    void clearValues();
    std::vector<QDoubleSpinBox*> values_;
    QSignalMapper* signal_mapper_;
};

}


#endif // ITOMP_INTERFACE_ITOMP_COST_FUNCTION_WIDGET_H