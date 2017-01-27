#ifndef ITOMP_NLP_CRF_FACTOR_MANAGER_H
#define ITOMP_NLP_CRF_FACTOR_MANAGER_H


#include <itomp_nlp/crf/crf_node.h>
#include <itomp_nlp/crf/crf_factor.h>

#include <map>


namespace itomp
{

class CRFFactorManager
{
private:

    CRFFactorManager();

public:

    static void registerFactor(const std::vector<CRFNode*>& nodes, CRFFactor* factor);
    static CRFFactor* getFactor(const std::vector<CRFNode*>& nodes);

private:

    static std::string getId(const std::vector<CRFNode*>& nodes);

    static std::map<std::string, CRFFactor*> factors_;
};

}


#endif // ITOMP_NLP_CRF_FACTOR_MANAGER_H