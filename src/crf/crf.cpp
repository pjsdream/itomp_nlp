#include <itomp_nlp/crf/crf.h>


namespace itomp_nlp
{

CRF::CRF()
{
}

void CRF::addExamples(CRFExamples* examples)
{
    examples_.push_back(examples);
}

void CRF::train()
{
    // collect available factors in the training dataset
    collectFactors();

    // gradient descent optimization
    dlib_dim_variables_ = 0;
    for (std::set<CRFFactor*>::iterator it = factors_.begin(); it != factors_.end(); it++)
        dlib_dim_variables_ += (*it)->dimFeatures();

    column_vector x(dlib_dim_variables_);
    dlib::find_min(dlib::bfgs_search_strategy(),
                   dlib::gradient_norm_stop_strategy(1e-6, 1000),
                   std::bind(&CRF::dlibCost, this, std::placeholders::_1),
                   std::bind(&CRF::dlibGradient, this, std::placeholders::_1),
                   x,
                   0.);

    // now that x is optimizied, update the final value to factors
    dlibUpdateVariablesToFactors(x);
}

void CRF::collectFactors()
{
    factors_.clear();

    for (int i=0; i<examples_.size(); i++)
    {
        std::set<CRFFactor*> factors = examples_[i]->getFactors();
        factors_.insert(factors.begin(), factors.end());
    }
}

double CRF::cost()
{
    double cost = 0.;

    for (int i=0; i<examples_.size(); i++)
        cost += examples_[i]->cost();

    // TODO: 2-norm regularization factor

    return cost;
}

Eigen::VectorXd CRF::gradient()
{
    // initialize factor gradients to zero
    int dim = 0;
    for (std::set<CRFFactor*>::iterator it = factors_.begin(); it != factors_.end(); it++)
    {
        CRFFactor* factor = *it;

        factor->setGradientZero();
        dim += factor->dimFeatures();
    }

    // add gradients for each examples to the cache
    // thread unsafe, unless mutex is used
    for (int i=0; i<examples_.size(); i++)
        examples_[i]->addGradient();

    // collect gradients
    Eigen::VectorXd gradient(dim);
    int idx = 0;

    for (std::set<CRFFactor*>::iterator it = factors_.begin(); it != factors_.end(); it++)
    {
        CRFFactor* factor = *it;
        const int factor_dim = factor->dimFeatures();

        gradient.block(idx, 0, factor_dim, 1) = factor->getWeight();
    }
    
    // TODO: 2-norm regularization factor

    return gradient;
}

void CRF::dlibUpdateVariablesToFactors(const column_vector& x)
{
    int idx = 0;

    for (std::set<CRFFactor*>::iterator it = factors_.begin(); it != factors_.end(); it++)
    {
        CRFFactor* factor = *it;

        // takes double pointer of raw array
        factor->setWeight(x.begin() + idx);
        
        const int dim = factor->dimFeatures();
        idx += dim;
    }
}

double CRF::dlibCost(const column_vector& x)
{
    dlibUpdateVariablesToFactors(x);

    return cost();
}

const CRF::column_vector CRF::dlibGradient(const column_vector& x)
{
    dlibUpdateVariablesToFactors(x);

    Eigen::VectorXd g = gradient();

    // convert eigen to dlib vector
    column_vector dlib_gradient(g.rows());
    memcpy(dlib_gradient.begin(), g.data(), sizeof(double) * g.rows());
    return dlib_gradient;
}

void CRF::infer(CRFExamples* example)
{
    // TODO: dynamic programming for finding the best RV assignments
}

void CRF::load(std::string directory)
{
    truncateTrailingDirectorySlash(directory);

    // TODO
}

void CRF::save(std::string directory)
{
    truncateTrailingDirectorySlash(directory);
    
    // TODO
}

void CRF::truncateTrailingDirectorySlash(std::string& string)
{
    if (string[ string.length()-1 ] == '/' || string[ string.length()-1 ] == '\\')
        string.resize( string.length()-1 );
}

}
