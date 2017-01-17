#ifndef ITOMP_NLP_CRF_H
#define ITOMP_NLP_CRF_H


#include <vector>

#include <itomp_nlp/crf/crf_examples.h>


namespace itomp_nlp
{

class CRF
{
public:

    CRF();

    void addExamples(CRFExamples* examples);

    void train();

    void infer(CRFExamples* graph);

    void load(std::string directory);
    void save(std::string directory);

private:

    void truncateTrailingDirectorySlash(std::string& directory);

    std::vector<CRFExamples*> examples_;
};

}


#endif // ITOMP_NLP_CRF_H