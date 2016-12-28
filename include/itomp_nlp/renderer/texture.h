#ifndef ITOMP_RENDERER_TEXTURE_H
#define ITOMP_RENDERER_TEXTURE_H


namespace itomp_renderer
{

class Texture
{
public:

    Texture(int id);

    inline int getId()
    {
        return texture_id_;
    }

private:

    int texture_id_;
};

}


#endif // ITOMP_RENDERER_TEXTURE_H