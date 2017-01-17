#include <itomp_nlp/crf/crf.h>


namespace itomp_nlp
{

CRF::CRF()
{
}

void CRF::addExamples(CRFExamples* examples)
{
    examples_.push_back(examples);
}

void CRF::train()
{
    // TODO: gradient descent-based method for convex problem
    // TODO: dynamic programming for computing normalization factor in tree-like factor graph
}

void CRF::infer(CRFExamples* example)
{
    // TODO: dynamic programming for finding the best RV assignments
}

void CRF::load(std::string directory)
{
    truncateTrailingDirectorySlash(directory);

    // TODO
}

void CRF::save(std::string directory)
{
    truncateTrailingDirectorySlash(directory);
    
    // TODO
}

void CRF::truncateTrailingDirectorySlash(std::string& string)
{
    if (string[ string.length()-1 ] == '/' || string[ string.length()-1 ] == '\\')
        string.resize( string.length()-1 );
}

}
