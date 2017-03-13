#ifndef ITOMP_NLP_CRF_GROUNDINGS_FACTOR_H
#define ITOMP_NLP_CRF_GROUNDINGS_FACTOR_H


#include <itomp_nlp/crf/crf_factor.h>


namespace itomp
{

class CRFGroundingsFactor : public CRFFactor
{
public:

    CRFGroundingsFactor();

    virtual double cost(const std::vector<CRFNode*>& nodes);
    virtual void addGradient(const std::vector<CRFNode*>& nodes);

private:

    void computeFeatures(const std::vector<CRFNode*>& nodes);
    Eigen::VectorXd features_;
};

}


#endif // ITOMP_NLP_CRF_GROUNDINGS_FACTOR_H