#include <itomp_nlp/optimization/cost.h>
#include <itomp_nlp/optimization/optimizer.h>


namespace itomp_optimization
{

Cost::Cost(Optimizer& optimizer, double weight)
    : optimizer_(optimizer)
    , weight_(weight)
{
}

}