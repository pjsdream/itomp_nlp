#ifndef ITOMP_NLP_CRF_NODE_H
#define ITOMP_NLP_CRF_NODE_H


#include <Eigen/Dense>

#include <vector>


namespace itomp_nlp
{

class CRFNode
{
public:

    CRFNode();

    virtual const std::string id() { return "Node"; }

private:

};

}


#endif // ITOMP_NLP_CRF_NODE_H