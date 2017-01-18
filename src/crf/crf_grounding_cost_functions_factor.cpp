#include <itomp_nlp/crf/crf_grounding_cost_functions_factor.h>


namespace itomp_nlp
{

CRFGroundingCostFucntionsFactor::CRFGroundingCostFucntionsFactor()
    : CRFFactor()
{
}

double CRFGroundingCostFucntionsFactor::cost(const std::vector<CRFNode*>& nodes)
{
    // TODO
    return 0.;
}

void CRFGroundingCostFucntionsFactor::addGradient(const std::vector<CRFNode*>& nodes)
{
    // TODO
}

}
