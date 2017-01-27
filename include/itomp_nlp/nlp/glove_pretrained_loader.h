#ifndef ITOMP_NLP_GLOVE_PRETRAINED_LOADER_H
#define ITOMP_NLP_GLOVE_PRETRAINED_LOADER_H


#include <itomp_nlp/nlp/word_to_vector.h>


namespace itomp
{

class GlovePretrainedLoader
{
public:

    GlovePretrainedLoader();

    WordToVector* loadGlovePretrainedData(const std::string& filename);

private:
};

}


#endif // ITOMP_NLP_GLOVE_PRETRAINED_LOADER_H