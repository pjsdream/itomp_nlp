#include <itomp_nlp/interface/itomp_cost_functions_widget.h>


namespace itomp
{

ItompCostFunctionsWidget::ItompCostFunctionsWidget(QWidget* parent)
    : QWidget(parent)
{
    layout_ = new QGridLayout(this);
    layout_->setRowStretch(100, 1);
    setLayout(layout_);

    add_button_ = new QPushButton("add", this);
    connect(add_button_, SIGNAL(clicked()), this, SLOT(addCostFunction()));

    layout_->addWidget(add_button_, 0, 0);
}

void ItompCostFunctionsWidget::addCostFunction()
{
    ItompCostFunctionWidget* cost_function_widget = new ItompCostFunctionWidget(widget_id_counts_, this);
    widget_id_counts_++;
    cost_function_widgets_.push_back(cost_function_widget);

    connect(cost_function_widget, SIGNAL(valueChanged(int, const std::string&, std::vector<double>)), 
            this, SIGNAL(costFunctionChanged(int, const std::string&, std::vector<double>)));

    layout_->addWidget( cost_function_widget, cost_function_widgets_.size(), 0 );

    // default cost function type
    cost_function_widget->selectCostType(0);
}

}
