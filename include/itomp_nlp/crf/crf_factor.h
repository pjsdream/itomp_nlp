#ifndef ITOMP_NLP_CRF_FACTOR_H
#define ITOMP_NLP_CRF_FACTOR_H


#include <Eigen/Dense>

#include <vector>

#include <itomp_nlp/crf/crf_node.h>


namespace itomp
{

class CRFFactor
{
public:

    CRFFactor();

    int dimFeatures();

    // array p to weights
    void setWeight(const double* p);

    inline const Eigen::VectorXd& getWeight()
    {
        return weights_;
    }

    virtual double cost(const std::vector<CRFNode*>& nodes) = 0;
    virtual void addGradient(const std::vector<CRFNode*>& nodes) = 0;

    void setGradientZero();

protected:

    // supposed to be initialized in derived classes
    Eigen::VectorXd weights_;
    Eigen::VectorXd weights_gradient_;
};

}


#endif // ITOMP_NLP_CRF_FACTOR_H