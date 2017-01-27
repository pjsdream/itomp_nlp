#include <itomp_nlp/crf/crf_factor.h>


namespace itomp
{

CRFFactor::CRFFactor()
{
}

int CRFFactor::dimFeatures()
{
    return weights_.rows();
}

void CRFFactor::setWeight(const double* p)
{
    for (int i=0; i<weights_.rows(); i++)
        weights_[i] = p[i];
}

void CRFFactor::setGradientZero()
{
    weights_gradient_.setZero();
}

}
