#ifndef ITOMP_NLP_CRF_FACTOR_H
#define ITOMP_NLP_CRF_FACTOR_H


#include <Eigen/Dense>

#include <vector>


namespace itomp_nlp
{

class CRFFactor
{
public:

    CRFFactor();

private:

    Eigen::VectorXd weights_;
};

}


#endif // ITOMP_NLP_CRF_FACTOR_H