#include <itomp_nlp/crf/crf_factor_manager.h>


namespace itomp_nlp
{

std::vector<CRFFactor*> CRFFactorManager::factors_;

CRFFactorManager::CRFFactorManager()
{
}

void CRFFactorManager::registerFactor(const std::vector<CRFNode*>& nodes, CRFFactor* factor)
{
    // TODO
}

CRFFactor* CRFFactorManager::getFactor(const std::vector<CRFNode*>& nodes)
{
    // TODO
    return 0;
}

}
