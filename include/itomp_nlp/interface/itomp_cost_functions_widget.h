#ifndef ITOMP_INTERFACE_ITOMP_COST_FUNCTIONS_WIDGET_H
#define ITOMP_INTERFACE_ITOMP_COST_FUNCTIONS_WIDGET_H


#include <QWidget>
#include <QGridLayout>
#include <QPushButton>

#include <itomp_nlp/optimization/optimizer.h>

#include <itomp_nlp/interface/itomp_cost_function_widget.h>


namespace itomp
{

class ItompCostFunctionsWidget : public QWidget
{
    Q_OBJECT

public:

    ItompCostFunctionsWidget(QWidget* parent = 0);
    
Q_SIGNALS:

    void costFunctionChanged(int id, const std::string& cost_type, std::vector<double> coefficients);

public Q_SLOTS:

    void addCostFunction();

private:

    Optimizer* optimizer_;

    QGridLayout* layout_;

    QPushButton* add_button_;

    std::vector<ItompCostFunctionWidget*> cost_function_widgets_;
    int widget_id_counts_;
};

}


#endif // ITOMP_INTERFACE_ITOMP_COST_FUNCTIONS_WIDGET_H