#include <itomp_nlp/crf/crf_phrase_grounding_factor.h>


namespace itomp_nlp
{

CRFPhraseGroundingFactor::CRFPhraseGroundingFactor()
    : CRFFactor()
{
}

double CRFPhraseGroundingFactor::cost(const std::vector<CRFNode*>& nodes)
{
    // TODO
    return 0.;
}

void CRFPhraseGroundingFactor::addGradient(const std::vector<CRFNode*>& nodes)
{
    // TODO
}

}
