#ifndef ITOMP_NLP_COMMANDS_TO_COST_H
#define ITOMP_NLP_COMMANDS_TO_COST_H


#include <itomp_nlp/nlp/word_to_vector.h>


namespace itomp
{

class CommandsToCost
{
public:

    CommandsToCost(const WordToVector* similarity_measure);

    void analyzeCosts(const std::string& natural_language_command);

private:

    const WordToVector* similarity_measure_;
};

}


#endif // ITOMP_NLP_COMMANDS_TO_COST_H