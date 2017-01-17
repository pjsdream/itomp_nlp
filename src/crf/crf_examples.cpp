#include <itomp_nlp/crf/crf_examples.h>


namespace itomp_nlp
{

CRFExamples::CRFExamples()
{
}

void CRFExamples::setNodeSize(int num_nodes, int num_factors)
{
    num_nodes_ = num_nodes;
    num_factors_ = num_factors;

    edge_node_to_factor_ = std::vector<std::vector<int> >(num_nodes);
    edge_factor_to_node_ = std::vector<std::vector<int> >(num_factors);
}

void CRFExamples::addEdge(int node_id, int factor_id)
{
    edge_node_to_factor_[node_id].push_back(factor_id);
    edge_factor_to_node_[factor_id].push_back(node_id);
}

void CRFExamples::addExample(bool is_positive, const std::vector<CRFNode*>& nodes)
{
    example_is_positive_.push_back(is_positive);
    example_nodes_.push_back(nodes);
}

}
