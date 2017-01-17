#ifndef ITOMP_NLP_CRF_FACTOR_MANAGER_H
#define ITOMP_NLP_CRF_FACTOR_MANAGER_H


#include <itomp_nlp/crf/crf_node.h>
#include <itomp_nlp/crf/crf_factor.h>


namespace itomp_nlp
{

class CRFFactorManager
{
private:

    CRFFactorManager();

public:

    static void registerFactor(const std::vector<CRFNode*>& nodes, CRFFactor* factor);
    static CRFFactor* getFactor(const std::vector<CRFNode*>& nodes);

private:

    static std::vector<CRFFactor*> factors_;
};

}


#endif // ITOMP_NLP_CRF_FACTOR_MANAGER_H