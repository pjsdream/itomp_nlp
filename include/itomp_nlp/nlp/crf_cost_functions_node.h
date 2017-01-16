#ifndef ITOMP_NLP_CRF_COST_FUNCTIONS_NODE_H
#define ITOMP_NLP_CRF_COST_FUNCTIONS_NODE_H


#include <itomp_nlp/nlp/crf_node.h>


namespace itomp_nlp
{

class CRFCostFunctionsNode : public CRFNode
{
public:

    CRFCostFunctionsNode();

    virtual const std::string id() { return "CostFunctions"; }

private:

};

}


#endif // ITOMP_NLP_CRF_COST_FUNCTIONS_NODE_H