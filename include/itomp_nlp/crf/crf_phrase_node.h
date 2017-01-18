#ifndef ITOMP_NLP_CRF_PHRASE_NODE_H
#define ITOMP_NLP_CRF_PHRASE_NODE_H


#include <itomp_nlp/crf/crf_node.h>


namespace itomp_nlp
{

class CRFPhraseNode : public CRFNode
{
public:

    CRFPhraseNode(bool is_known);

    virtual const std::string id() { return "Phrase"; }

private:

};

}


#endif // ITOMP_NLP_CRF_PHRASE_NODE_H