#include <itomp_nlp/crf/crf_factor_manager.h>


namespace itomp
{

std::map<std::string, CRFFactor*> CRFFactorManager::factors_;


CRFFactorManager::CRFFactorManager()
{
}

void CRFFactorManager::registerFactor(const std::vector<CRFNode*>& nodes, CRFFactor* factor)
{
    std::string id = getId(nodes);
    factors_[id] = factor;
}

CRFFactor* CRFFactorManager::getFactor(const std::vector<CRFNode*>& nodes)
{
    std::string id = getId(nodes);

    std::map<std::string, CRFFactor*>::iterator it = factors_.find(id);

    if (it == factors_.end())
        return 0;

    return it->second;
}

std::string CRFFactorManager::getId(const std::vector<CRFNode*>& nodes)
{
    std::string id;

    for (int i=0; i<nodes.size(); i++)
    {
        id += nodes[i]->id();
        if (i + 1 < nodes.size())
            id += "; ";
    }

    return id;
}

}
