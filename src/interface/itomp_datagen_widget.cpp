#define _USE_MATH_DEFINES
 
#include <itomp_nlp/interface/itomp_datagen_widget.h>


namespace itomp
{

ItompDatagenWidget::ItompDatagenWidget(QWidget* parent)
    : QWidget(parent)
{
    setWindowTitle("Motion planner");

    resize(600, 600);

    layout_ = new QGridLayout(this);
    setLayout(layout_);

    random_pose_button_ = new QPushButton("Reset", this);
    connect(random_pose_button_, SIGNAL(clicked()), this, SLOT(setRandomPose()));

    layout_->addWidget(random_pose_button_, 0, 0);

    itomp_cost_functions_widget_ = new ItompCostFunctionsWidget(this);
    connect(itomp_cost_functions_widget_, SIGNAL(costFunctionChanged(int, const std::string&, std::vector<double>)),
            this, SLOT(costFunctionChanged(int, const std::string&, std::vector<double>)));
    
    scroll_area_ = new QScrollArea(this);
    scroll_area_->setWidget(itomp_cost_functions_widget_);
    scroll_area_->setWidgetResizable(true);

    layout_->setRowStretch(1, 1);
    layout_->addWidget(scroll_area_, 1, 0);
    
    // initialize resource
    initializeResources();
}

void ItompDatagenWidget::initializeResources()
{
}

}
