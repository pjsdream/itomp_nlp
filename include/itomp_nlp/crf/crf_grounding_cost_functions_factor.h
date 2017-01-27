#ifndef ITOMP_NLP_CRF_GROUNDING_COST_FUNCTIONS_FACTOR_H
#define ITOMP_NLP_CRF_GROUNDING_COST_FUNCTIONS_FACTOR_H


#include <itomp_nlp/crf/crf_factor.h>


namespace itomp
{

class CRFGroundingCostFucntionsFactor : public CRFFactor
{
public:

    CRFGroundingCostFucntionsFactor();

    virtual double cost(const std::vector<CRFNode*>& nodes);
    virtual void addGradient(const std::vector<CRFNode*>& nodes);
};

}


#endif // ITOMP_NLP_CRF_GROUNDING_COST_FUNCTIONS_FACTOR_H