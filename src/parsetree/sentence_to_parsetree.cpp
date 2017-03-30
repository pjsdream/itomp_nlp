#include <itomp_nlp/parsetree/sentence_to_parsetree.h>


namespace itomp
{

const std::string SentenceToParsetree::grammar_ =
"S -> NP VP\n"
"PP -> P NP\n"
"NP -> Det N | Det N PP | \'I\'\n"
"VP -> V NP | VP PP\n"
"Det -> \'an\' | \'my\'\n"
"N -> \'elephant\' | \'pajamas\'\n"
"V -> \'shot\'\n"
"P -> \'in\'"
;

SentenceToParsetree::SentenceToParsetree()
{
}

}
