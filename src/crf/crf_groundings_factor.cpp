#include <itomp_nlp/crf/crf_groundings_factor.h>


namespace itomp
{

CRFGroundingsFactor::CRFGroundingsFactor()
    : CRFFactor()
{
}

double CRFGroundingsFactor::cost(const std::vector<CRFNode*>& nodes)
{
    computeFeatures(nodes);

    return weights_.dot(features_);
}

void CRFGroundingsFactor::addGradient(const std::vector<CRFNode*>& nodes)
{
    // TODO
}

void CRFGroundingsFactor::computeFeatures(const std::vector<CRFNode*>& nodes)
{
    // TODO
}

}
