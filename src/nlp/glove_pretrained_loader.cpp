#include <itomp_nlp/nlp/glove_pretrained_loader.h>

#include <stdio.h>

#include <Eigen/Dense>

#include <itomp_nlp/utils/timing.h>


namespace itomp_nlp
{

GlovePretrainedLoader::GlovePretrainedLoader()
{
}

WordToVector* GlovePretrainedLoader::loadGlovePretrainedData(const std::string& filename)
{
    // Takes ~8 sec in home desktop

    WordToVector* dictionary = new WordToVector();

    FILE* fp = fopen(filename.c_str(), "r");
    if (fp == NULL)
        return dictionary;
    
    itomp_utils::timerStart();

    // parse vector dimension
    // filename format: glove.[#tokens].[dimension].txt
    std::string dimension_string = filename;
    if (filename.find_last_of("/\\") != std::string::npos)
        dimension_string = filename.substr( filename.find_last_of("/\\") + 1 );
    dimension_string = dimension_string.substr( 0, dimension_string.find_last_of('.') - 1 );
    dimension_string = dimension_string.substr( dimension_string.find_last_of('.') + 1 );
    const int dimension = std::stoi(dimension_string);

    char word[1024];
    Eigen::Matrix<double, 50, 1> v;
    while (fscanf(fp, "%s", word) == 1)
    {
        for (int i=0; i<dimension; i++)
            fscanf(fp, "%lf", &v(i));

        dictionary->addMapping(word, v);
    }
    
    itomp_utils::timerPrintElapsedTime();

    fclose(fp);
}

}
