#ifndef ITOMP_NLP_CRF_H
#define ITOMP_NLP_CRF_H


#include <vector>
#include <set>

#include <itomp_nlp/crf/crf_examples.h>

#include <dlib/optimization.h>


namespace itomp_nlp
{

class CRF
{
private:

    typedef dlib::matrix<double, 0, 1> column_vector;

public:

    CRF();

    void addExamples(CRFExamples* examples);

    void train();

    void infer(CRFExamples* graph);

    void load(std::string directory);
    void save(std::string directory);

private:

    void truncateTrailingDirectorySlash(std::string& directory);

    double cost();
    Eigen::VectorXd gradient();
    void collectFactors();
    std::set<CRFFactor*> factors_;

    // dlib optimization interfaces
    int dlib_dim_variables_;
    void dlibUpdateVariablesToFactors(const column_vector& x);
    double dlibCost(const column_vector& x);
    const column_vector dlibGradient(const column_vector& x);

    std::vector<CRFExamples*> examples_;
};

}


#endif // ITOMP_NLP_CRF_H