#ifndef ITOMP_RENDERER_MODEL_TEXTURE_H
#define ITOMP_RENDERER_MODEL_TEXTURE_H


namespace itomp_renderer
{

class ModelTexture
{
public:

    ModelTexture(int id);

    inline int getId()
    {
        return texture_id_;
    }

private:

    int texture_id_;
};

}


#endif // ITOMP_RENDERER_MODEL_TEXTURE_H