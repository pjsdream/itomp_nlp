#ifndef ITOMP_NLP_CRF_EXAMPLES_H
#define ITOMP_NLP_CRF_EXAMPLES_H


#include <vector>
#include <set>

#include <itomp_nlp/crf/crf_node.h>
#include <itomp_nlp/crf/crf_factor.h>


namespace itomp
{

// Has one graph structure and multiple examples, each of which is either positive or negative
// An example has assignments to all random variables
class CRFExamples
{
public:

    CRFExamples();

    void setNodeSize(int num_nodes, int num_factors);
    void setNegativeExampleCoefficient(double lambda);

    std::set<CRFFactor*> getFactors();

    // Edge insertion order is important. Order matters in sharing factor weights.
    // Factors identify the order of nodes and their types
    void addEdge(int node_id, int factor_id);

    void addExample(bool is_positive, const std::vector<CRFNode*>& nodes);

    double cost();
    void addGradient();

private:

    double negative_example_coefficient_;

    int num_nodes_;
    int num_factors_;
    std::vector<std::vector<int> > edge_node_to_factor_;
    std::vector<std::vector<int> > edge_factor_to_node_;

    std::vector<char> example_is_positive_;
    std::vector<std::vector<CRFNode*> > example_nodes_;
};

}


#endif // ITOMP_NLP_CRF_EXAMPLES_H