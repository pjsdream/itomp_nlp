#ifndef ITOMP_NLP_WORD_TO_VECTORS_H
#define ITOMP_NLP_WORD_TO_VECTORS_H


#include <Eigen/Dense>

#include <map>
#include <vector>


namespace itomp_nlp
{

class WordToVector
{
public:

    WordToVector();

    void addMapping(const std::string& str, const Eigen::VectorXd& v);

    Eigen::VectorXd getVector(const std::string& str);

    double cosineSimilarity(const std::string& word1, const std::string& word2);

private:
    
    int dimension_;
    std::map<std::string, Eigen::VectorXd> map_;
};

}


#endif // ITOMP_NLP_WORD_TO_VECTORS_H