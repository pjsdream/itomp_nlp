#ifndef ITOMP_NLP_CRF_NODE_H
#define ITOMP_NLP_CRF_NODE_H


#include <Eigen/Dense>

#include <vector>


namespace itomp_nlp
{

class CRFNode
{
public:

    CRFNode(bool is_known);

    virtual const std::string id() { return "Node"; }

protected:

    bool is_known_;

};

}


#endif // ITOMP_NLP_CRF_NODE_H