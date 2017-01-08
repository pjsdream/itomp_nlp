#include <itomp_nlp/interface/itomp_cost_function_widget.h>


namespace itomp_interface
{

const std::vector<std::pair<std::string, int> > ItompCostFunctionWidget::cost_types_ = 
{
    {"none", 1},
    {"smoothness", 1},
    {"goal", 4},
};

ItompCostFunctionWidget::ItompCostFunctionWidget(int id, QWidget* parent)
    : QWidget(parent)
    , id_(id)
{
    layout_ = new QGridLayout(this);
    setLayout(layout_);

    combo_box_ = new QComboBox(this);
    for (int i=0; i<cost_types_.size(); i++)
        combo_box_->addItem( cost_types_[i].first.c_str() );
    connect(combo_box_, SIGNAL(activated(int)), this, SLOT(selectCostType(int)));

    layout_->addWidget(combo_box_, 0, 0);
}

void ItompCostFunctionWidget::selectCostType(int index)
{
    clearValues();

    for (int i=0; i<cost_types_[index].second; i++)
    {
        QDoubleSpinBox* double_spin_box = new QDoubleSpinBox(this);
        double_spin_box->setRange(-100., 100.);
        double_spin_box->setDecimals(6);
        double_spin_box->setSingleStep(1e-6);
        connect(double_spin_box, SIGNAL(editingFinished()), this, SLOT(slotValueChanged()));

        values_.push_back(double_spin_box);

        layout_->setColumnStretch(i+1, 1);
        layout_->addWidget(double_spin_box, 0, i + 1);
    }
}

void ItompCostFunctionWidget::clearValues()
{
    for (int i=0; i<values_.size(); i++)
    {
        layout_->removeWidget(values_[i]);
        layout_->setColumnStretch(i+1, 0);
        delete values_[i];
    }

    values_.clear();
}

void ItompCostFunctionWidget::slotValueChanged()
{
    const int combo_box_index = combo_box_->currentIndex();

    std::vector<double> coefficients;
    for (int i=0; i<values_.size(); i++)
        coefficients.push_back( values_[i]->value() );

    emit valueChanged(id_, cost_types_[combo_box_index].first, coefficients);
}

}
