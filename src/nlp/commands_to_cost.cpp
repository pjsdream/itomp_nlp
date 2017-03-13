#include <itomp_nlp/nlp/commands_to_cost.h>


namespace itomp
{

CommandsToCost::CommandsToCost(const WordToVector* similarity_measure)
    : similarity_measure_(similarity_measure)
{
}

void CommandsToCost::analyzeCosts(const std::string& natural_language_command)
{
    // TODO: find verb using CoreNLP tokens
    const std::string verb = natural_language_command.substr( 0, natural_language_command.find_first_of(' ') );

    // TODO: define seed words in other place
    static std::vector<std::string> seed_words = 
    {
        "move",
        "pull",
        "push",
        "pick",
        "grasp",
        "place",
        "up",
        "down",
        "left",
        "right",
        "fast",
        "slow",
    };

    printf("Word <%s>\n", verb.c_str());
    printf("similarities to seed words:\n");
    for (int i=0; i<seed_words.size(); i++)
    {
        printf("%9.6lf <%s>\n", similarity_measure_->cosineSimilarity(verb, seed_words[i]), seed_words[i].c_str());
    }
}

}
