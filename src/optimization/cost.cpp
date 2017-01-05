#include <itomp_nlp/optimization/cost.h>
#include <itomp_nlp/optimization/optimizer_thread.h>


namespace itomp_optimization
{

Cost::Cost(OptimizerThread& optimizer, double weight)
    : optimizer_(optimizer)
    , weight_(weight)
{
}

}