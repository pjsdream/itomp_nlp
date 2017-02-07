#include <QApplication>
#include <cstdlib>
#include <stdio.h>

#include <itomp_nlp/nlp/glove_pretrained_loader.h>
#include <itomp_nlp/nlp/commands_to_cost.h>

#include <itomp_nlp/interface/main_window.h>

#include <itomp_nlp/renderer/rendering_kinect_human.h>


int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    /*
    itomp_nlp::GlovePretrainedLoader glove_pretrained_loader;
#ifdef WIN32
    itomp_nlp::WordToVector* word_to_vector = glove_pretrained_loader.loadGlovePretrainedData("C:\\lib\\glove.6B\\glove.6B.50d.txt");
#else
    itomp_nlp::WordToVector* word_to_vector = glove_pretrained_loader.loadGlovePretrainedData("/playpen/jaesungp/lib/glove.6B/glove.6B.50d.txt");
#endif

    itomp_nlp::CommandsToCost commands_to_cost(word_to_vector);
    commands_to_cost.analyzeCosts("move");
    commands_to_cost.analyzeCosts("pull");
    commands_to_cost.analyzeCosts("push");
    commands_to_cost.analyzeCosts("thrust");
    commands_to_cost.analyzeCosts("slow");
    commands_to_cost.analyzeCosts("vertically");
    commands_to_cost.analyzeCosts("horizontally");

    delete word_to_vector;
    */

    QApplication app(argc, argv);
    itomp::MainWindow* main_window = new itomp::MainWindow();
    main_window->show();
    
    /*
    itomp::Renderer* renderer = new itomp::Renderer();
    
    itomp::Material* material = new itomp::Material();
    material->setDiffuseColor(Eigen::Vector4f(0.5, 0.5, 0.5, 1));

    itomp::RenderingKinectHuman* kinect_human = new itomp::RenderingKinectHuman(renderer);
    kinect_human->setMaterial(material);
    renderer->addShape(kinect_human);

    renderer->show();
    */

    app.exec();

    return 0;
}
