#include <itomp_nlp/crf/crf_examples.h>

#include <itomp_nlp/crf/crf_factor_manager.h>


namespace itomp
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

void CRFExamples::setNegativeExampleCoefficient(double lambda)
{
    negative_example_coefficient_ = lambda;
}

std::set<CRFFactor*> CRFExamples::getFactors()
{
    std::set<CRFFactor*> factors;

    for (int i=0; i<example_nodes_.size(); i++)
    {
        for (int j=0; j<num_factors_; j++)
        {
            std::vector<CRFNode*> factor_nodes;
            for (int k=0; k<edge_factor_to_node_[j].size(); k++)
                factor_nodes.push_back(example_nodes_[i][ edge_factor_to_node_[j][k] ]);

            CRFFactor* factor = CRFFactorManager::getFactor(factor_nodes);
            factors.insert(factor);
        }
    }

    return factors;
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

double CRFExamples::cost()
{
    double cost = 0.;

    for (int i=0; i<example_nodes_.size(); i++)
    {
        for (int j=0; j<num_factors_; j++)
        {
            std::vector<CRFNode*> factor_nodes;
            for (int k=0; k<edge_factor_to_node_[j].size(); k++)
                factor_nodes.push_back(example_nodes_[i][ edge_factor_to_node_[j][k] ]);

            CRFFactor* factor = CRFFactorManager::getFactor(factor_nodes);
            const double factor_cost = factor->cost(factor_nodes);

            if (example_is_positive_[i])
                cost += factor_cost;
            else
                cost -= negative_example_coefficient_ * factor_cost;
        }
    }

    return cost;
}

void CRFExamples::addGradient()
{
    for (int i=0; i<example_nodes_.size(); i++)
    {
        for (int j=0; j<num_factors_; j++)
        {
            std::vector<CRFNode*> factor_nodes;
            for (int k=0; k<edge_factor_to_node_[j].size(); k++)
                factor_nodes.push_back(example_nodes_[i][ edge_factor_to_node_[j][k] ]);

            CRFFactor* factor = CRFFactorManager::getFactor(factor_nodes);
            factor->addGradient(factor_nodes);
        }
    }
}

}
