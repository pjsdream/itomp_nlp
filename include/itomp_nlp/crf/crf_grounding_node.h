#ifndef ITOMP_NLP_CRF_GROUNDING_NODE_H
#define ITOMP_NLP_CRF_GROUNDING_NODE_H


#include <itomp_nlp/crf/crf_node.h>


namespace itomp
{

class CRFGroundingNode : public CRFNode
{
public:

    CRFGroundingNode(bool is_known);

    virtual const std::string id() { return "Grounding"; }

private:

};

}


#endif // ITOMP_NLP_CRF_GROUNDING_NODE_H