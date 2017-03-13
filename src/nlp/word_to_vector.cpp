#include <itomp_nlp/nlp/word_to_vector.h>

#include <iostream>


namespace itomp
{

WordToVector::WordToVector()
{
}

void WordToVector::addMapping(const std::string& str, const Eigen::VectorXd& v)
{
    map_[str] = v;
    dimension_ = v.rows();

    if (map_.size() % 10000 == 0)
        printf("reading %d words\n", map_.size());
}

Eigen::VectorXd WordToVector::getVector(const std::string& str) const
{
    std::map<std::string, Eigen::VectorXd>::const_iterator it = map_.find(str);

    if (it == map_.cend())
    {
        fprintf(stderr, "ERROR: word vector for <%s> not found\n", str.c_str());
        // TODO: handle exceptions
        return Eigen::VectorXd(dimension_);
    }

    return it->second;
}

double WordToVector::cosineSimilarity(const std::string& word1, const std::string& word2) const
{
    const Eigen::VectorXd v1 = getVector(word1);
    const Eigen::VectorXd v2 = getVector(word2);

    if (v1.isZero() || v2.isZero())
        return -1.;

    return v1.dot(v2) / (v1.norm() * v2.norm());
}

}
