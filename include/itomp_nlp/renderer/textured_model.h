#ifndef ITOMP_RENDERER_TEXTURED_MODEL_H
#define ITOMP_RENDERER_TEXTURED_MODEL_H


#include <itomp_nlp/renderer/raw_model.h>
#include <itomp_nlp/renderer/model_texture.h>


namespace itomp_renderer
{

class TexturedModel
{
public:

    TexturedModel(RawModel* model, ModelTexture* texture);

    inline RawModel* getModel()
    {
        return raw_model_;
    }

    inline ModelTexture* getTexture()
    {
        return texture_;
    }

private:

    RawModel* raw_model_;
    ModelTexture* texture_;
};

}


#endif // ITOMP_RENDERER_TEXTURED_MODEL_H