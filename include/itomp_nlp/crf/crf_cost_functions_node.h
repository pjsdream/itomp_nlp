#ifndef ITOMP_NLP_CRF_COST_FUNCTIONS_NODE_H
#define ITOMP_NLP_CRF_COST_FUNCTIONS_NODE_H


#include <itomp_nlp/crf/crf_node.h>


namespace itomp
{

class CRFCostFunctionsNode : public CRFNode
{
public:

    CRFCostFunctionsNode(bool is_known);

    virtual const std::string id() { return "CostFunctions"; }

private:

};

}


#endif // ITOMP_NLP_CRF_COST_FUNCTIONS_NODE_H