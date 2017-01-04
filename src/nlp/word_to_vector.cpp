#include <itomp_nlp/nlp/word_to_vector.h>


namespace itomp_nlp
{

WordToVector::WordToVector()
{
}

void WordToVector::addMapping(const std::string& str, const Eigen::VectorXd& v)
{
    map_[str] = v;
    dimension_ = v.rows();
}

Eigen::VectorXd WordToVector::getVector(const std::string& str)
{
    std::map<std::string, Eigen::VectorXd>::iterator it = map_.find(str);

    if (it == map_.end())
    {
        // TODO: handle exceptions
        return Eigen::VectorXd(dimension_);
    }

    return it->second;
}

double WordToVector::cosineSimilarity(const std::string& word1, const std::string& word2)
{
    Eigen::VectorXd v1 = getVector(word1);
    Eigen::VectorXd v2 = getVector(word2);

    return v1.dot(v2) / (v1.norm() * v2.norm());
}

}
